################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_dlu9576)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  turtlesim
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  TurtleStatus.msg
)

add_service_files(
  FILES
  ControlStatus.srv
  LogRate.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  turtlesim
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  roscpp
  rospy
  std_msgs
  turtlesim
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/turtlesim_control
  nodes/turtlesim_logger
  nodes/turtlesim_monitor
  nodes/turtlesim_pygame
  scripts/turtlesim_control_srv
  scripts/turtlesim_lograte_srv
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
