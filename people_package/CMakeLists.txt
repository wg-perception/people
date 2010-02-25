cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#rosbuild_genmsg()
rosbuild_gensrv()
#find_library(curl REQUIRED)


rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#rosbuild_add_executable(timing_diagnostics_node
#                       src/timing_diagnostics_node.cpp )
#rosbuild_link_boost(timing_diagnostics_node thread)

#rosbuild_add_executable(face_detector
#                       src/face_detection.cpp 
#                       src/people.cpp 
#                       src/utils.cpp)
#rosbuild_link_boost(face_detector thread)


#rosbuild_add_executable(stereo_face_color_tracker
#                       src/stereo_face_color_tracker.cpp 
#                       src/people.cpp 
#                       src/utils.cpp)
#rosbuild_link_boost(stereo_face_color_tracker thread)

#rosbuild_add_executable(stereo_color_tracker
#		      src/stereo_color_tracker.cpp)
#rosbuild_link_boost(stereo_color_tracker thread)

#rosbuild_add_executable(track_starter_gui
#                       src/track_starter_gui.cpp 
#                       src/utils.cpp)
#rosbuild_link_boost(track_starter_gui thread)



find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

#rosbuild_add_library(pyface_detector
#                    src/py.cpp src/face_detection.cpp src/people.cpp)
#set_target_properties(pyface_detection PROPERTIES OUTPUT_NAME face_detection PREFIX "")

rosbuild_add_executable(leg_detector 
                       src/laser_processor.cpp
                       src/leg_detector.cpp 
                       src/calc_leg_features.cpp)


rosbuild_add_executable(train_leg_detector
                       src/laser_processor.cpp
                       src/train_leg_detector.cpp 
                       src/calc_leg_features.cpp)


#rosbuild_add_pyunit(test/directed.py)

rosbuild_add_executable(height_tracker
                        src/height_tracker.cpp)


#rosbuild_download_test_data(http://pr.willowgarage.com/data/people_package/face_detector_noface_test.bag face_detector_noface_test.bag 6dce1a1b06fd43c37fdb34f754d38632)
#rosbuild_download_test_data(http://pr.willowgarage.com/data/people_package/face_detector_withface_test.bag face_detector_withface_test.bag e81af59b9f649c9615d2a83a7d12b784)
#rosbuild_add_rostest(test/face_detector_true_rtest.xml)
#rosbuild_add_rostest(test/face_detector_false_rtest.xml)