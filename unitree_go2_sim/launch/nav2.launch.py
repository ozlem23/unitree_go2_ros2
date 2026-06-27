import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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

    # ─── Paket dizinleri (nav2_config_path'ten ÖNCE tanımlanmalı) ───
    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')

    # ─── Konfigürasyon dosyaları ───
    joints_config      = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(unitree_go2_sim, "config/ros_control/ros_control.yaml")
    gait_config        = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config       = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    nav2_config_path   = os.path.join(unitree_go2_sim, "config", "navigation", "nav2_params.yaml")

    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(unitree_go2_description, "worlds/default.sdf")

    # ─── Launch argümanları ───
    declare_use_sim_time      = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_rviz              = DeclareLaunchArgument("rviz", default_value="true")
    declare_robot_name        = DeclareLaunchArgument("robot_name", default_value="go2")
    declare_ros_control_file  = DeclareLaunchArgument("ros_control_file", default_value=ros_control_config)
    declare_gazebo_world      = DeclareLaunchArgument("world", default_value=default_world_path)
    declare_world_init_x      = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y      = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z      = DeclareLaunchArgument("world_init_z", default_value="0.375")
    declare_world_init_heading= DeclareLaunchArgument("world_init_heading", default_value="0.0")
    declare_description_path  = DeclareLaunchArgument(
        "unitree_go2_description_path", default_value=default_model_path)
    declare_map_yaml = DeclareLaunchArgument(
        "map", default_value="/root/my_map.yaml",
        description="Statik haritanin tam yolu")

    # ─── Robot Description ───
    robot_description = {
        "robot_description": Command([
            "xacro ", LaunchConfiguration("unitree_go2_description_path"),
            " robot_controllers:=", LaunchConfiguration("ros_control_file")
        ])
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ─── CHAMP Kontrolcüleri ───
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

    # ─── EKF Düğümleri ───
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
            {"odom0_config": [False, False, False, False, False, False,
                               True,  True,  False, False, False, True,
                               False, False, False]},
            {"imu0": "imu/data"},
            {"imu0_config": [False, False, False, False, False, True,
                              False, False, False, False, False, True,
                              False, False, False]},
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # ─── Gazebo ───
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                unitree_go2_description, 'worlds', 'default.sdf'
            ]), ' -r']
        }.items(),
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
            '-Y', LaunchConfiguration('world_init_heading'),
        ],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'lazy': False}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
    )

    # ─── 3D→2D LiDAR Köprüsü ───
    pc2_to_laserscan_bridge_node = Node(
        package='unitree_go2_sim',
        executable='pc2_to_laserscan_node',
        name='pc2_to_laserscan_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
        }]
    )
    # # ─── Nav2 Bringup (harita + AMCL + navigasyon) - GECİKTİRMELİ ───
    # delayed_nav2_navigation = TimerAction(
    #     period=7.0,  # Gazebo ve EKF'in kendine gelmesi için 5 saniye bekle
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
    #             ),
    #             launch_arguments={
    #                 'use_sim_time': use_sim_time,
    #                 'map': LaunchConfiguration('map'),
    #                 'params_file': nav2_config_path,
    #                 'autostart': 'true',
    #                 'initial_pose_x': LaunchConfiguration('world_init_x'),
    #                 'initial_pose_y': LaunchConfiguration('world_init_y'),
    #                 'initial_pose_yaw': LaunchConfiguration('world_init_heading'),
    #             }.items()
    #         )
    #     ]
    # )
# ─── Sadece Başlangıç Pozunu Gecikmeli Gönderen Tetikleyici ───
    delayed_initial_pose = TimerAction(
        period=7.0,  # Nav2'nin ve AMCL'in tamamen ayağa kalkması için beklenen süre
        actions=[
            Node(
                package='ros2topic',
                executable='ros2topic',
                name='init_pose_publisher',
                arguments=[
                    'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                    [
                        '{header: {frame_id: "map"}, pose: {pose: {position: {x: ', 
                        LaunchConfiguration('world_init_x'), 
                        ', y: ', 
                        LaunchConfiguration('world_init_y'), 
                        ', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
                    ]
                ],
                output='screen'
            )
        ]
    )
# ─── Nav2 Bringup (harita + AMCL + navigasyon) ───
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map'),
            'params_file': nav2_config_path,
            'autostart': 'true',
            # Nav2 / AMCL için otomatik başlangıç konumu parametreleri:
            'initial_pose_x': LaunchConfiguration('world_init_x'),
            'initial_pose_y': LaunchConfiguration('world_init_y'),
            'initial_pose_yaw': LaunchConfiguration('world_init_heading'),
        }.items()
    )
    # ─── AMCL Konusunun Hazır Olmasını Bekleyen ve Pozu Gönderen Akıllı Süreç ───
    # Bu script, AMCL konusunun açılmasını bekler, açıldığı an pozu basar ve kapanır.
# ─── AMCL ve Nav2 İçin Güvenli Başlangıç Pozu Tetikleyici ───
    smart_initial_pose = ExecuteProcess(
        cmd=[
            'python3', '-c',
            """
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
import time

# Argümanları terminal parametresi olarak güvenle alıyoruz
x_val = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
y_val = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

rclpy.init()
node = rclpy.create_node('smart_init_pose_node')
publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

node.get_logger().info('AMCL ve Nav2 baslangici icin bekliyor (8 saniye)...')
time.sleep(8.0) 

msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.pose.pose.position.x = x_val
msg.pose.pose.position.y = y_val
msg.pose.pose.position.z = 0.0
msg.pose.pose.orientation.w = 1.0

# AMCL'in veriyi kacirmamasi icin 3 kez arka arkaya gonderiyoruz
for i in range(3):
    msg.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(msg)
    node.get_logger().info(f'Baslangic pozu AMCLye iletildi (X: {x_val}, Y: {y_val}) - Deneme {i+1}')
    time.sleep(0.5)

node.destroy_node()
rclpy.shutdown()
"""
        , 
        # Python betiğine LaunchConfiguration değerlerini ROS 2 alt yapısıyla güvenli şekilde argüman geçiyoruz:
        LaunchConfiguration('world_init_x'), 
        LaunchConfiguration('world_init_y')
        ],
        output='screen',
        shell=False # Shell=False yaparak argüman listesini koruyoruz
    )
    # ─── Controller Spawner'lar ───
    controller_spawner_js = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["--controller-manager-timeout", "120", "joint_states_controller"],
            parameters=[{"use_sim_time": use_sim_time}],
        )]
    )

    controller_spawner_effort = TimerAction(
        period=15.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["--controller-manager-timeout", "120", "joint_group_effort_controller"],
            parameters=[{"use_sim_time": use_sim_time}],
        )]
    )

    # ─── RViz ───
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
        declare_map_yaml,

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
        nav2_navigation,
        smart_initial_pose,
        rviz2,
    ])