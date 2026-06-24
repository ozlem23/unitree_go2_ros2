cmake_minimum_required(VERSION 3.8)
project(unitree_go2_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. BAĞIMLILIKLARI BUL (C++ ROS 2 Kütüphaneleri)
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. C++ DÜĞÜMÜNÜ DERLE VE BAĞIMLILIKLARI BAĞLA
# =========================================================
add_executable(pc2_to_laserscan_node src/pointcloud_to_laserscan_bridge.cpp)

# Hem rclcpp hem de sensor_msgs paketlerinin include ve library yollarını tek seferde bağlar (Çakışmayı önler)
ament_target_dependencies(pc2_to_laserscan_node rclcpp sensor_msgs)

# 3. KLASÖRLERİ VE C++ ÇIKTILARINI SİSTEME YÜKLE (INSTALL)
# =========================================================
# DİKKAT: Hedef pc2_to_laserscan_node'dur (Node adı launch dosyalarında çağrılır)
install(TARGETS
  pc2_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
)

# Klasörleri share/ altına yükler
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()