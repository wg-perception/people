cmake_minimum_required(VERSION 2.8.3)
project(people_velocity_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  people_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs people_msgs
)
