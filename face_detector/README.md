# face_detector

See [the ROS wiki]http://wiki.ros.org/face_detector) for full information.

## Note about classifier files
The package is a ROS wrapper around OpenCV's face detection algorithm. You need to provide a classifier configuration file for it to work, specified as the `classifier_filename` ROS parameter.

OpenCV provides a number of different options. On Ubuntu they are stored in `/usr/share/opencv4/haarcascades`. This location is stored in the `default_folder` file. This package will attempt to load the file specified in `classifier_filename` both as an absolute path, and as a path relative to the `default_folder` value.
