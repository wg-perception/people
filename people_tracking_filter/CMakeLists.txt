cmake_minimum_required(VERSION 2.8.3)
project(people_tracking_filter)

# Look for bfl (Bayesian Filtering Library)
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  people_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        geometry_msgs
        message_filters
        people_msgs
        roscpp
        sensor_msgs
        std_msgs
        tf
    LIBRARIES people_tracking_filter
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS}/bfl
)

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
add_dependencies(people_tracking_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(people_tracking_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})

add_executable(people_tracker src/people_tracking_node.cpp)
add_dependencies(people_tracker people_tracking_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(people_tracker
   people_tracking_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslaunch_add_file_check(launch)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS people_tracking_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS people_tracker
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
