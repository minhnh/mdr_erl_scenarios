cmake_minimum_required(VERSION 2.8.3)
project(mdr_erl_scenarios)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  mas_execution_manager
  mdr_demo_simple_pick_and_place
  atwork_ros_msgs
)

# setup Python package
catkin_python_setup()

# package specific setup, will be more relevant for C++ code, but required nonetheless
catkin_package()

roslint_python()

# install executables
#install(PROGRAMS
#  scripts/talker_node
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# install launch files
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
#)

