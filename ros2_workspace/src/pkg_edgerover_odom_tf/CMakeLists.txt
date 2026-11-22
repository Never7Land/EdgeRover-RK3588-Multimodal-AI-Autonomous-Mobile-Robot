cmake_minimum_required(VERSION 3.8)
project(pkg_edgerover_odom_tf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom_tf_node src/odom_tf_node.cpp)
ament_target_dependencies(odom_tf_node rclcpp nav_msgs geometry_msgs tf2_ros)

install(TARGETS odom_tf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
