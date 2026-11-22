cmake_minimum_required(VERSION 3.8)
project(edgerover_mission)

# 与其他自研包保持一致：C++17 + 相同的警告选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
# 找到 ros_bridge 包生成的 VlmAsk 服务头文件
find_package(edgerover_ros_bridge REQUIRED)

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node
  rclcpp std_msgs geometry_msgs sensor_msgs vision_msgs edgerover_ros_bridge)

# 装到 lib/edgerover_mission 下，ros2 run 才能找到
install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})
ament_package()