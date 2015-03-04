cmake_minimum_required(VERSION 2.8.3)
project(people_tracking_filter)

# Look for bfl (Bayesian Filtering Library)
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  message_filters
  people_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)

## Declare things to be passed to dependent projects
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS
    sensor_msgs
    std_msgs
    geometry_msgs
    people_msgs
LIBRARIES people_tracking_filter
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(people_tracking_filter 
            src/uniform_vector.cpp 
            src/gaussian_vector.cpp 
            src/gaussian_pos_vel.cpp 
            src/mcpdf_pos_vel.cpp
            src/mcpdf_vector.cpp
            src/sysmodel_pos_vel.cpp
            src/sysmodel_vector.cpp
            src/measmodel_pos.cpp
            src/measmodel_vector.cpp
	    src/tracker_particle.cpp 
	    src/tracker_kalman.cpp 
	    src/detector_particle.cpp 
)

## Declare a cpp executable
add_executable(people_tracker src/people_tracking_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(people_tracker people_msgs_gencpp people_tracking_filter)

## Specify libraries to link a library or executable target against
target_link_libraries(people_tracker
   people_tracking_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES}
)

target_link_libraries(people_tracking_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS people_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
