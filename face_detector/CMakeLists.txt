cmake_minimum_required(VERSION 3.0.2)
project(face_detector)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        cv_bridge
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_generation
        people_msgs
        rosbag
        roscpp
        roslib
        rospy
        sensor_msgs
        std_msgs
        std_srvs
        stereo_msgs
        tf
)

find_package(Boost REQUIRED COMPONENTS signals system thread)
find_package(OpenCV)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY action
  FILES FaceDetector.action
)
generate_messages(DEPENDENCIES actionlib_msgs people_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        actionlib
        actionlib_msgs
        cv_bridge
        geometry_msgs
        image_geometry
        image_transport
        message_filters
        message_runtime
        people_msgs
        roscpp
        roslib
        rospy
        sensor_msgs
        std_msgs
        stereo_msgs
        tf
)

add_executable(face_detector
                       src/face_detection.cpp
                       src/faces.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_dependencies(face_detector ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS
  face_detector
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/face_detector/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_download_test_data(${PROJECT_NAME}_noface_test_diamondback.bag
    http://download.ros.org/data/face_detector/face_detector_noface_test_diamondback.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 37f043be780a4511c853379defdd9855)
  catkin_download_test_data(${PROJECT_NAME}_withface_test_diamondback.bag
    http://download.ros.org/data/face_detector/face_detector_withface_test_diamondback.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 59126117e049e69d577b7ee27251a6f8)

#  add_rostest(test/wide-stereo_true_rtest.xml)
#  add_rostest(test/wide-stereo_false_rtest.xml)
  add_rostest(test/rgbd_true_rtest.xml)
  add_rostest(test/rgbd_false_rtest.xml)
#  add_rostest(test/narrow-stereo_true_rtest.xml)
#  add_rostest(test/narrow-stereo_false_rtest.xml)
#  add_rostest(test/action-wide_true_rtest.xml)
#  add_rostest(test/action-wide_false_rtest.xml)
#  add_rostest(test/action-rgbd_true_rtest.xml)
#  add_rostest(test/action-rgbd_false_rtest.xml)

endif()
