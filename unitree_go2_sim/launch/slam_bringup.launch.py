import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = "base_footprint"  

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(package="unitree_go2_description").find("unitree_go2_description")
    
    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(unitree_go2_sim, "config/ros_control/ros_control.yaml")
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(unitree_go2_description, "worlds/default.sdf")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="true")
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="go2")
    declare_ros_control_file = DeclareLaunchArgument("ros_control_file", default_value=ros_control_config)
    declare_gazebo_world = DeclareLaunchArgument("world", default_value=default_world_path)
    
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")
    declare_description_path = DeclareLaunchArgument("unitree_go2_description_path", default_value=default_model_path)
    
    robot_description = {"robot_description": Command(["xacro ", LaunchConfiguration("unitree_go2_description_path"),
                                                       " robot_controllers:=", LaunchConfiguration("ros_control_file")])}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"}, 
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"close_loop_odom": False},
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')])},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            os.path.join(get_package_share_directory("champ_base"), "config", "ekf", "base_to_footprint.yaml"),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_link_frame": base_frame},
            {"odom_frame": "odom"},
            {"world_frame": "odom"},
            {"publish_tf": True},
            {"frequency": 50.0},
            {"two_d_mode": True},
            {"odom0": "odom/raw"},
            {"odom0_config": [False, False, False, False, False, False, True, True, False, False, False, True, False, False, False]},
            {"imu0": "imu/data"},
            {"imu0_config": [False, False, False, False, False, True, False, False, False, False, False, True, False, False, False]},
        ],
        remappings=[("odometry/filtered", "odom")],
    )
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [PathJoinSubstitution([unitree_go2_description, 'worlds', 'default.sdf']), ' -r']}.items(),
    )
    
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )
    
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'lazy': False,
        }],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',          # GZ -> ROS
            #'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            #'/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            
            # DOĞRU YÖN: Eklem durumları Gazebo'dan ROS'a akmalı ([)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # 3D ham veri — bu topic korunuyor, kaybolmuyor
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            
            # DOĞRU YÖN: Odometri verisi Gazebo'dan ROS'a akmalı ([)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',     # ROS -> GZ
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory', # ROS -> GZ
        ],
    )

    local_slam_config = os.path.join(
        unitree_go2_sim, "config", "slam_toolbox", "mapper_params_online_async.yaml"
    )

    # =========================================================================
    # DÜZELTME 1: target_frame 'base_link' olmalı, 'velodyne' değil.
    # velodyne frame'i robot hareket edince titrer, base_link sabit referans noktası.
    # cloud_in: /velodyne_points/points (3D ham veri buradan alınır, SİLİNMEZ)
    # scan:     /scan (2D dönüştürülmüş çıktı, slam_toolbox bunu okur)
    # =========================================================================
# 3D LiDAR'ı 2D Scan'e Dönüştüren C++ Köprü Düğümü
    pc2_to_laserscan_bridge_node = Node(
        package='unitree_go2_sim',
        executable='pc2_to_laserscan_node',
        name='pc2_to_laserscan_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_link',  # Bu satırı ekliyoruz
            'transform_tolerance': 0.05   # Zaman senkronizasyonu için tolerans payı
        }]
    )

    # =========================================================================
    # DÜZELTME 2: async → sync.
    # sync_slam_toolbox_node lifecycle gerektirmez, direkt active başlar.
    # Yanlış remap kaldırıldı (/scan zaten /scan'e gidiyordu, gereksizdi).
    # YAML'da scan_topic: /scan olduğundan emin ol!
    # =========================================================================
# sync yerine tekrar async_slam_toolbox_node kullanalım çünkü yaml dosyan async için yapılandırılmış.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # Tekrar async yaptık
        name='slam_toolbox',
        output='screen',
        parameters=[
            local_slam_config,
            {'use_sim_time': True},
        ],
    )
    # SLAM Toolbox'ı otomatik olarak CONFIGURE ve ACTIVATE eden yönetici düğüm
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True, # İşte sihirli kelime! Kendisi configure ve activate yapacak.
            'node_names': ['slam_toolbox'] # Yukarıda tanımladığın name='slam_toolbox' ile eşleşmeli
        }]
    )
    controller_spawner_js = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["--controller-manager-timeout", "120", "joint_states_controller"],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    controller_spawner_effort = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_group_effort_controller",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_robot_name,
        declare_ros_control_file,
        declare_gazebo_world,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,
        declare_description_path, 
        
        gz_sim,
        robot_state_publisher_node,
        gazebo_spawn_robot,
        gazebo_bridge,
        
        quadruped_controller_node,
        state_estimator_node,
        
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        
        controller_spawner_js,
        controller_spawner_effort,
        
        pc2_to_laserscan_bridge_node,
        slam_toolbox_node,
        slam_lifecycle_manager,
        rviz2,
    ])