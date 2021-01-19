cmake_minimum_required(VERSION 2.8.3)
project(leg_detector)

# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)
link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
pkg_check_modules(BULLET bullet)

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

include_directories(
  include ${catkin_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS}
)

add_executable(leg_detector
               src/laser_processor.cpp
               src/leg_detector.cpp
               src/calc_leg_features.cpp)

add_dependencies(leg_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(leg_detector
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslaunch_add_file_check(launch)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS leg_detector
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/trained_leg_detector.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
