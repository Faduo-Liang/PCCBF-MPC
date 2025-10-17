cmake_minimum_required(VERSION 3.0.2)
project(scene)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scene
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(movetest_node src/movetest.cpp)

target_link_libraries(movetest_node
  ${catkin_LIBRARIES}
)

add_executable(getrobot_pose src/getrobot_pose.cpp)

target_link_libraries(getrobot_pose
  ${catkin_LIBRARIES}
)

add_executable(pseudo_odom src/pseudo_odom.cpp)

target_link_libraries(pseudo_odom
  ${catkin_LIBRARIES}
)
