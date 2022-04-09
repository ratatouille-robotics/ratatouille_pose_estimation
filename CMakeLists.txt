cmake_minimum_required(VERSION 3.0.2)
project(ratatouille_pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  LIBRARIES ratatouille_pose_estimation
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
