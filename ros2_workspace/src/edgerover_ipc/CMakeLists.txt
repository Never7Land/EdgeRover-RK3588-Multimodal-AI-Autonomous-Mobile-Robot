cmake_minimum_required(VERSION 3.8) # 指定 CMake 最低版本
project(edgerover_ipc)              # 定义项目名称

set(CMAKE_CXX_STANDARD 17)          # 使用 C++17 标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 强制要求使用指定的 C++ 标准
add_compile_options(-Wall -Wextra -Wpedantic -O2) # 开启编译警告并启用 O2 优化

find_package(ament_cmake REQUIRED)  # 查找 ROS2 的 ament_cmake 构建工具

# 核心库：以后相机、YOLO、VLM 进程都会链接它

add_library(frame_ring src/frame_ring.cpp) # 将 frame_ring.cpp 编译成 frame_ring 库

target_include_directories(frame_ring PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 编译时使用项目内的 include 目录
  $<INSTALL_INTERFACE:include>)                           # 安装后使用安装目录下的 include

target_link_libraries(frame_ring PUBLIC rt) # 链接 rt 库，提供 shm_open 等共享内存相关接口

# 两个测试程序：模拟相机（写端）和模拟 YOLO（读端）

add_executable(demo_writer src/demo_writer.cpp) # 编译生成模拟相机写入程序
target_link_libraries(demo_writer frame_ring)   # 链接核心共享内存库

add_executable(demo_reader src/demo_reader.cpp) # 编译生成模拟 YOLO 读取程序
target_link_libraries(demo_reader frame_ring)   # 链接核心共享内存库

install(DIRECTORY include/ DESTINATION include) # 安装头文件到安装目录的 include 下

# 安装核心库，并导出库的目标信息，供其他 ROS2 功能包使用

install(TARGETS frame_ring EXPORT export_edgerover_ipc
  ARCHIVE DESTINATION lib  # 静态库安装目录
  LIBRARY DESTINATION lib  # 动态库安装目录
  RUNTIME DESTINATION bin) # 可执行运行文件安装目录

# 可执行文件放在 lib/<包名> 下，ros2 run 才能找到

install(TARGETS demo_writer demo_reader DESTINATION lib/${PROJECT_NAME})

# 导出核心库，让其他 ROS2 功能包可以通过 find_package(edgerover_ipc) 使用

ament_export_targets(export_edgerover_ipc HAS_LIBRARY_TARGET)

ament_package() # 生成 ROS2 功能包所需的配置和元数据，完成 CMake 配置