cmake_minimum_required(VERSION 3.0.2)
project(harvester_arm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES harvester_arm
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/pick_and_place_gripper.py
  scripts/pick_and_place.py
  scripts/wiggle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)