################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_small_warehouse_world)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
# find_package(gazebo REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################


################################################################################
# Build
################################################################################

#include_directories(
#  include
#)
#set(EXEC_NAME "amazon_robot_drive")

#add_executable(${EXEC_NAME} src/amazon_robot_drive.cpp)
#ament_target_dependencies(${EXEC_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch models rviz worlds
        DESTINATION share/${PROJECT_NAME}/
        )



ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_package()