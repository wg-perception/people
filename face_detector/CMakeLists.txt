cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib)
include(${actionlib_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(face_detector
                       src/face_detection.cpp 
                       src/people.cpp 
                       src/utils.cpp)
rosbuild_link_boost(face_detector thread)

rosbuild_add_executable(face_detector_action
                       src/face_detection_action.cpp 
                       src/people.cpp 
                       src/utils.cpp)
rosbuild_link_boost(face_detector_action thread)



rosbuild_download_test_data(http://pr.willowgarage.com/data/face_detector/face_detector_noface_test.bag face_detector_noface_test.bag 6dce1a1b06fd43c37fdb34f754d38632)
rosbuild_download_test_data(http://pr.willowgarage.com/data/face_detector/face_detector_withface_test.bag face_detector_withface_test.bag e81af59b9f649c9615d2a83a7d12b784)
rosbuild_add_rostest(test/face_detector_true_rtest.xml)
rosbuild_add_rostest(test/face_detector_false_rtest.xml)
