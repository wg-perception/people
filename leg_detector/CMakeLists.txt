cmake_minimum_required(VERSION 3.0.2)
project(leg_detector)

# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    image_geometry
    laser_geometry
    message_filters
    people_msgs
    people_tracking_filter
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

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        message_filters
        people_msgs
        people_tracking_filter
        roscpp
        sensor_msgs
        std_msgs
        tf
        visualization_msgs
)

add_executable(leg_detector
               src/laser_processor.cpp
               src/leg_detector.cpp
               src/calc_leg_features.cpp)

add_dependencies(leg_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(leg_detector
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

install(TARGETS leg_detector
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/trained_leg_detector.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
