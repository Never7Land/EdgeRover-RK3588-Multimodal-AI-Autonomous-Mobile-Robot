cmake_minimum_required(VERSION 3.8)
project(edgerover_ros_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(edgerover_ipc REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VlmAsk.srv"
  "msg/Stats.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(ros_bridge src/ros_bridge.cpp)
target_include_directories(ros_bridge PRIVATE ${ZMQ_INCLUDE_DIRS})

ament_target_dependencies(ros_bridge rclcpp std_msgs vision_msgs sensor_msgs)
target_link_libraries(ros_bridge nlohmann_json::nlohmann_json ${ZMQ_LIBRARIES} "${cpp_typesupport_target}" edgerover_ipc::frame_ring)

install(TARGETS ros_bridge DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()