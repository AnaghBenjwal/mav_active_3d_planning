cmake_minimum_required(VERSION 3.0.2)
project(copter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  std_msgs
  visualization_msgs
)


find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)



include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES FastPlannerOctomap
#  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs octomap octomap_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

# add all the files to be compiled
#name of the cpp fiel
add_executable(Controller Controller.cpp)
target_link_libraries(Controller ${catkin_LIBRARIES})




