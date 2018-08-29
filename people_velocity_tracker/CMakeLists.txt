cmake_minimum_required(VERSION 2.8.3)
project(people_velocity_tracker)

find_package(catkin REQUIRED COMPONENTS
  easy_markers
  geometry_msgs
  kalman_filter
  people_msgs
  roslib
  rospy
)

catkin_package(
    CATKIN_DEPENDS 
        easy_markers geometry_msgs kalman_filter people_msgs roslib rospy
)

catkin_install_python(PROGRAMS scripts/static.py scripts/tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES 
  launch/filtered_tracked_detector.launch
  launch/tracked_detector.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
