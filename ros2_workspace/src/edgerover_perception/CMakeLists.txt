cmake_minimum_required(VERSION 3.8)
project(edgerover_perception LANGUAGES C CXX)   # 同时编译 C 和 C++

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(edgerover_ipc REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Qt5 REQUIRED COMPONENTS Widgets)   # 全屏预览用 Qt5 Widgets（EGLFS 平台插件随 qtbase5-dev 安装）

# RGA：头文件来自 SDK（放在本包的 third_party 目录下），运行库使用板子上已有的 librga.so.2
set(RGA_INC ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rga/include)

add_library(rk_vendor STATIC
  vendor_c/common.c
  vendor_c/dev_Camera.c
  vendor_c/rga_helper.c)
target_compile_definitions(rk_vendor PRIVATE _GNU_SOURCE)

# 两个路径都加：一个对应 #include <rga/RgaApi.h>，一个对应 #include "im2d.h"
target_include_directories(rk_vendor PUBLIC vendor_c ${RGA_INC} ${RGA_INC}/rga)

# 直接写完整路径：系统里没有 librga.so 这个软链接，-lrga 找不到
target_link_libraries(rk_vendor PUBLIC /usr/lib/aarch64-linux-gnu/librga.so.2)

# 相机进程：采集 → RGA → 写入 FrameRing
add_executable(camera_proc src/camera_proc.cpp)
target_link_libraries(camera_proc rk_vendor edgerover_ipc::frame_ring)

# 调试工具：从 FrameRing 读取一帧，保存成 jpg
add_executable(frame_dump src/frame_dump.cpp)
target_link_libraries(frame_dump edgerover_ipc::frame_ring ${OpenCV_LIBS})
target_include_directories(frame_dump PRIVATE ${OpenCV_INCLUDE_DIRS})

# 全屏摄像头预览：只读 FrameRing，不打开 V4L2（camera_proc 独占 /dev/video44）
add_executable(camera_view src/camera_view.cpp)
target_link_libraries(camera_view Qt5::Widgets edgerover_ipc::frame_ring)

install(TARGETS camera_proc frame_dump DESTINATION lib/${PROJECT_NAME})
install(TARGETS camera_proc frame_dump camera_view DESTINATION lib/${PROJECT_NAME})   # 新增 camera_view，ros2 run 和 systemd 都要这个安装路径
ament_package()