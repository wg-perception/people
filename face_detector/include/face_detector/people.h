/*********************************************************************
* People-specific computer vision algorithms.
*
**********************************************************************
*
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Caroline Pantofaru
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef PEOPLE_H
#define PEOPLE_H


/**
@mainpage

@htmlinclude manifest.html

@b "people" is a package containing algorithms and nodes related to the perception of people. 
This may include detection, tracking, recognition, activity recognition, etc. 
people.h and people.cpp provide the algorithms, while the other files provide wrapper ROS nodes.

@section usage Usage
See launch/face_dector.wide.launch to launch in continuous detection mode, or
launch/face_detector_action.wide.launch to launch as an action.

@param classifier_name A readable string for the classifier. Will be published with the result.
@param classifier_filename Full path to the trained haar cascade. Currently useful cascades are haar_frontalface_alt.xml haar_profileface.xml in opencv2 
@param classifier_reliability Double 0-1. Some notion of the classifier's reliability for use in a larger system.
@param do_continuous true=run continuously, false=wait for action call
@param do_publish_faces_of_unknown_size true-->If depth info is not available, just make pos.x and pos.y in the resulting PositionMeasurement messaging the 2D image center of the face. false-->Don't publish faces if stereo information isn't available.
@param do_display none/local Don't display anything / display in an OpenCV window.

 **/



#include <stdio.h>
#include <iostream>
#include <vector>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>

#include "image_geometry/stereo_camera_model.h"
#include "utils.h"
#include "ros/time.h"
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>


// Thresholds for the face detection algorithm.
#define FACE_SIZE_MIN_M 0.1
#define FACE_SIZE_MAX_M 0.5
#define MAX_Z_M 8
#define FACE_DIST 1.0 //0.4
// Timeouts
#define TRACKING_FILTER_TIMEOUT_S 5.0

using namespace std;

/**
   A structure containing the data related to one person.
 */
struct Person {
  string id;
  string name;
};


/**
   Contains a list of people and functions that can be performed on that list.
   This includes utility tasks such as set/get data, to more complicated tasks such as detection or tracking.
 */
class People
{
 public:

  // Create an empty list of people.
  People();

  // Destroy a list of people.
  ~People();


  /*!
   * \brief Detect all faces in an image.
   * 
   * Input:
   * \param image - The image in which to detect faces.
   * \param haar_classifier_filename - Path to the xml file containing the trained haar classifier cascade.
   * \param threshold - Detection threshold. Currently unused.
   * \param disparity_image - Image of disparities (from stereo). To avoid using depth information, set this to NULL.
   * \param cam_model - The camera model created by CvStereoCamModel.
   * \param do_draw - If true, draw a box on `image' around each face.
   * Output:
   * A vector of CvRects containing the bounding boxes around found faces.
   */ 
  vector<Box2D3D> detectAllFaces(IplImage *image, double threshold, IplImage *disparity_image, image_geometry::StereoCameraModel *cam_model);
  void initFaceDetection(uint num_cascades, string haar_classifier_filename);

 ////////////////////
 private:


  /**< The list of people. */
  vector<Person> list_;

  /**< Grayscale image (to avoid reallocating an image each time an OpenCV function is run.) */
  IplImage *cv_image_gray_;
  IplImage *disparity_image_;
  image_geometry::StereoCameraModel *cam_model_;

  boost::mutex face_mutex_, face_done_mutex_, t_mutex_;
  boost::mutex* face_go_mutex_;
  boost::thread_group threads_;
  vector<Box2D3D> faces_;
  boost::condition face_detection_ready_cond_, face_detection_done_cond_;
  int num_threads_to_wait_for_;
  int images_ready_;

  /* Structures for the face detector. */
  /**< Classifier cascade for face detection. */
  CvHaarClassifierCascade* cascade_;
  /**< Storage for OpenCV functions. */
  CvMemStorage* storage_;

  void faceDetectionThread(uint i);

};

#endif

