cmake_minimum_required(VERSION 2.8.3)
project(social_navigation_layers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  people_msgs
  dynamic_reconfigure
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/ProxemicLayer.cfg
)

catkin_package(
    CATKIN_DEPENDS people_msgs costmap_2d dynamic_reconfigure
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## add cpp library
add_library(social_layers 
            src/social_layer.cpp
            src/proxemic_layer.cpp 
            src/passing_layer.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(social_layers people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS social_layers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
