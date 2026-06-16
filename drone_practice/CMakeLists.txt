cmake_minimum_required(VERSION 3.0.2)
project(drone_practice)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    mavros_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/path.py
  scripts/dodge.py
  scripts/land.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
