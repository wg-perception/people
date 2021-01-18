# face_detector

See [the ROS wiki](http://wiki.ros.org/face_detector) for full information.

## Note about classifier files
The package is a ROS wrapper around OpenCV's face detection algorithm. You need to provide a classifier configuration file for it to work, specified as the `classifier_filename` ROS parameter.

Older versions of this package would only accept an absolute path. Now, if the configuration file is not found using the absolute path, it will use check the list of default folders. This can be provided using the `classifier_folders` ROS parameter, or if that is not set, will use the values in `param/default_folders.yaml`.
