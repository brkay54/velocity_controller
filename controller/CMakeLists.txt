cmake_minimum_required(VERSION 3.5)
project(controller)


if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lgsvl_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(talker src/longitudinal_controller.cpp)
ament_target_dependencies(talker rclcpp std_msgs lgsvl_msgs nav_msgs)

install(TARGETS
talker
DESTINATION lib/${PROJECT_NAME})

ament_package()
