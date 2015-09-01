^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package face_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2015-09-01)
------------------
* Install missing param directory: face_detector.rgbd.launch fails due to the `param` folder.
* Contributors: Isaac I.Y. Saito

1.0.8 (2014-12-10)
------------------
* cleanup formatting with astyle (supersedes `#18 <https://github.com/wg-perception/people/issues/18>`_)
* centrally reference cascade from standalone opencv (addresses `#15 <https://github.com/wg-perception/people/issues/15>`_)
* Contributors: Dan Lazewatsky

1.0.3 (2014-03-01)
------------------
* fix message generation
* Contributors: Dan Lazewatsky

1.0.2 (2014-02-28)
------------------
* update to properly generate messages
* Contributors: Dan Lazewatsky

1.0.1 (2014-02-27)
------------------
* update some remappings to deal with the changed openni message api
* convert 16FC1 depth images to 32FC1 and scale to get meters
* switch constants to defines to get rid of linker issues for catkin switch
* catkinizing
* Contributors: Dan Lazewatsky
