cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()	

rosbuild_gensrv()
rosbuild_genmsg()

rosbuild_add_boost_directories()

rosbuild_add_executable(bin/person_pos_sender src/person_node.cpp)
rosbuild_add_executable(bin/is_person_on_path src/is_person_on_path.cpp)
rosbuild_link_boost(bin/is_person_on_path thread)

rospack_add_lisp_executable(bin/hallway_move people-aware-nav lanes:main)

rospack_add_lisp_executable(bin/fake_person people-aware-nav fake-person-sender:fake-person-sender)

rospack_add_lisp_executable(bin/test_move_right people-aware-nav lanes:test-move-right)

rosbuild_add_executable(bin/head_controller src/head_controller.cpp)
rosbuild_link_boost(bin/head_controller thread)

rosbuild_add_library (navfn_constrained src/navfn_constrained.cpp)

rosbuild_check_for_display(disp)
if(disp)
  rosbuild_add_executable(bin/hallway_test test/hallway_test.cpp)
  rosbuild_add_gtest_build_flags(bin/hallway_test)
  rosbuild_add_rostest(test/test_move_right.launch)
endif(disp)
