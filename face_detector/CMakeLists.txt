cmake_minimum_required(VERSION 2.8.3)
project(face_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  cv_bridge
  stereo_msgs
  sensor_msgs
  tf
  rosbag
  image_transport
  people_msgs
  image_geometry
  genmsg
  actionlib_msgs)

find_package(Boost REQUIRED COMPONENTS signals system thread)
find_package(OpenCV)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_action_files(
  DIRECTORY action
  FILES FaceDetector.action
)
generate_messages(DEPENDENCIES actionlib_msgs people_msgs)

add_executable(face_detector
                       src/face_detection.cpp 
                       src/faces.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES})

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS people_msgs geometry_msgs)

install(TARGETS
  face_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/face_detector/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   catkin_download_test_data(${PROJECT_NAME}_face_detector_noface_test_diamondback.bag
#     http://download.ros.org/data/face_detector/face_detector_noface_test_diamondback.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 37f043be780a4511c853379defdd9855)
#   catkin_download_test_data(${PROJECT_NAME}_ace_detector_withface_test_diamondback.bag
#     http://download.ros.org/data/face_detector/face_detector_withface_test_diamondback.bag
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
#     MD5 59126117e049e69d577b7ee27251a6f8)

#   add_rostest(test/face_detector_true_rtest.xml)
#   add_rostest(test/face_detector_false_rtest.xml)
# endif()