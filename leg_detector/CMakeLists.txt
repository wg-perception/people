cmake_minimum_required(VERSION 2.8.3)
project(leg_detector)

# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  laser_geometry
  people_msgs
  people_tracking_filter
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/LegDetector.cfg
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    geometry_msgs
    people_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
  DEPENDS
    BFL
    BULLET
  LIBRARIES
    leg_laser_processing
)

## Library of functions used for laser -> leg features
add_library(leg_laser_processing
            src/laser_processor.cpp
            src/calc_leg_features.cpp)
add_dependencies(leg_laser_processing people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(leg_laser_processing
  ${catkin_LIBRARIES} ${BULLET_LIBRARIES}
)

## Declare a cpp executable
add_executable(leg_detector src/leg_detector.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(leg_detector leg_laser_processing)

## Specify libraries to link a library or executable target against
target_link_libraries(leg_detector
  leg_laser_processing ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

## Declare a cpp executable
add_executable(train_leg_detector src/train_leg_detector.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(train_leg_detector leg_laser_processing)

## Specify libraries to link a library or executable target against
target_link_libraries(train_leg_detector
  leg_laser_processing ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

install(
  TARGETS leg_laser_processing leg_detector train_leg_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/trained_leg_detector.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
