cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  visualization_msgs
  tf
  f110_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs f110_msgs
)

## Include directories

include_directories(include 
  /home/patrick/race_stack/f110_utils/nodes/frenet_odom_republisher/include
  ${catkin_INCLUDE_DIRS} 
)

## Declare a C++ executable for stanley_controller
add_executable(frenet_odom_republisher_node
    		src/frenet_odom_republisher_node.cc
    		src/frenet_conversion.cc
)
target_link_libraries(frenet_odom_republisher_node ${catkin_LIBRARIES})

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/lane_visualize.py
  scripts/static_obs_spline_planner.py
  scripts/static_obstacle_pub.py
  scripts/frenet_utils.py
  scripts/obstacle_detect_pnu.py
  scripts/tester.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
