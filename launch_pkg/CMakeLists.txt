cmake_minimum_required(VERSION 3.0.2)
project(launch_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rospy)

# add_message_files(
# 	DIRECTORY msg
# 	FILES
  
#   Control_rtc.msg
#   Status_rtc.msg

#   Control_main.msg
#   Status_main.msg

#   Status_oc.msg
#   Control_oc.msg

#   Status_hc.msg
#   Control_hc.msg

#   Status_cpd.msg
#   Control_cpd.msg

# )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
# generate_messages(DEPENDENCIES std_msgs geometry_msgs )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs )