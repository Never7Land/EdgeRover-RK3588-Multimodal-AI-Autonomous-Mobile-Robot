# 不含 C++ 代码，只把 launch 目录安装到 share/，供 ros2 launch 找到

cmake_minimum_required(VERSION 3.8)

project(edgerover_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()