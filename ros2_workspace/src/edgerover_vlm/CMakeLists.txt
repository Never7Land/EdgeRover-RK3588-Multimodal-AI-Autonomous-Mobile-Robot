cmake_minimum_required(VERSION 3.8)
project(edgerover_vlm)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(edgerover_ipc REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

set(TP ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# 服务端：常驻，持有 NPU
add_executable(vlm_proc src/vlm_proc.cpp src/image_enc.cc)
target_include_directories(vlm_proc PRIVATE src ${TP}/include ${OpenCV_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(vlm_proc ${OpenCV_LIBS} ${ZMQ_LIBRARIES}
  ${TP}/lib/librkllmrt.so ${TP}/lib/librknnrt.so)
ament_target_dependencies(vlm_proc edgerover_ipc)
set_target_properties(vlm_proc PROPERTIES INSTALL_RPATH "$ORIGIN")

# 客户端：命令行发一个问题
add_executable(vlm_ask src/vlm_ask.cpp)
target_include_directories(vlm_ask PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(vlm_ask ${ZMQ_LIBRARIES})

install(TARGETS vlm_proc vlm_ask DESTINATION lib/${PROJECT_NAME})
install(FILES ${TP}/lib/librkllmrt.so ${TP}/lib/librknnrt.so DESTINATION lib/${PROJECT_NAME})

ament_package()