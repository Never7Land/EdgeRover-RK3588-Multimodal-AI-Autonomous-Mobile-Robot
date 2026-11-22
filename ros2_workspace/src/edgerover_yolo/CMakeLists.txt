cmake_minimum_required(VERSION 3.8)
project(edgerover_yolo)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(edgerover_ipc REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# 复用 vlm 包里的 rknn_api.h
set(RKNN_INC ${CMAKE_CURRENT_SOURCE_DIR}/../edgerover_vlm/third_party/include)

add_executable(yolo_proc src/yolo_proc.cpp)
target_include_directories(yolo_proc PRIVATE ${RKNN_INC} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(yolo_proc ${ZMQ_LIBRARIES} /usr/lib/librknnrt.so)
ament_target_dependencies(yolo_proc edgerover_ipc)

add_executable(yolo_sub src/yolo_sub.cpp)
target_include_directories(yolo_sub PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(yolo_sub ${ZMQ_LIBRARIES})

install(TARGETS yolo_proc yolo_sub DESTINATION lib/${PROJECT_NAME})
ament_package()