cmake_minimum_required(VERSION 3.8)
project(netlink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(serial)
add_subdirectory(nlink_utils)


find_package(ament_cmake REQUIRED)
find_package(netlink_msg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(nlink_node)
target_sources(nlink_node
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/nlinktrack/src/protocols.cpp
        ${CMAKE_CURRENT_LIST_DIR}/nlinktrack/src/nlinktrack_module.cpp
        ${CMAKE_CURRENT_LIST_DIR}/nlinktrack/src/main.cpp
)
target_include_directories(nlink_node
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/nlinktrack/include
)
target_link_libraries(nlink_node
    nlink_utils
    serial
)
ament_target_dependencies(nlink_node rclcpp std_msgs netlink_msg)


ament_package()
