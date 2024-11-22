cmake_minimum_required(VERSION 3.0.2)
project(message_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rospy)

add_message_files(
  DIRECTORY msg
  FILES
  App_request.msg
  Vel_msg.msg
  Keyboard_command.msg
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES message_pkg
  CATKIN_DEPENDS message_runtime roscpp rospy
  # DEPENDS system_lib
)
