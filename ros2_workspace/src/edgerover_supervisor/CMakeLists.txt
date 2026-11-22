cmake_minimum_required(VERSION 3.8)
project(edgerover_supervisor)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(edgerover_ipc REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_executable(supervisor src/supervisor.cpp)
target_include_directories(supervisor PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(supervisor ${ZMQ_LIBRARIES})
ament_target_dependencies(supervisor edgerover_ipc)

install(TARGETS supervisor DESTINATION lib/${PROJECT_NAME})
ament_package()
