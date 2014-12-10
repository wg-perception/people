/*********************************************************************
* Face-specific computer vision algorithms.
*
**********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Caroline Pantofaru */

#ifndef FACES_H
#define FACES_H



#include <stdio.h>
#include <iostream>
#include <vector>

#include <opencv/cv.hpp>
#include <opencv/cxcore.hpp>
#include <opencv/cvaux.hpp>

#include "image_geometry/stereo_camera_model.h"
#include "ros/time.h"
#include <ros/console.h>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>


using namespace std;

#define FACE_SIZE_MIN_M 0.1 /**< Default minimum face size, in meters. Only use this for initialization. */
#define FACE_SIZE_MAX_M 0.5 /**< Default maximum face size, in meters. Only use this for initialization. */
#define MAX_FACE_Z_M    8.0 /**< Default maximum distance from the camera, in meters. Only use this for initialization. */
// Default thresholds for face tracking.
#define FACE_SEP_DIST_M 1.0 /**< Default separation distance for associating faces. Only use this for initialization. */

namespace people
{


/*!
 * \brief A structure for holding information about boxes in 2d and 3d.
 */
struct Box2D3D
{
  cv::Point2d center2d;
  cv::Point3d center3d;
  double width2d;
  double width3d;
  cv::Rect box2d;
  string status;
  int id;
};


/*!
 * \brief A structure containing the person's identifying data.
 */
struct Face
{
  string id;
  string name;
};


/*!
 * \brief Contains a list of faces and functions that can be performed on that list.
 * This includes utility tasks such as set/get data, to more complicated tasks such as detection or tracking.
 */

class Faces
{
public:

  // Default thresholds for the face detection algorithm.

  // Thresholds for the face detection algorithm.
  double face_size_min_m_; /**< Minimum face size, in meters. */
  double face_size_max_m_; /**< Maximum face size, in meters. */
  double max_face_z_m_; /**< Maximum distance from the camera, in meters. */
  // Thresholds for face tracking.
  double face_sep_dist_m_; /**< Separation distance for associating faces. */



  // Create an empty list of people.
  Faces();

  // Destroy a list of people.
  ~Faces();


  /*!
   * \brief Detect all faces in an image and disparity image.
   *
   * Input:
   * \param image The image in which to detect faces.
   * \param haar_classifier_filename Path to the xml file containing the trained haar classifier cascade.
   * \param threshold Detection threshold. Currently unused.
   * \param disparity_image Image of disparities (from stereo).
   * \param cam_model The camera model used to convert 2D points to 3D points.
   * Output:
   * A vector of Box2D3Ds containing the bounding boxes around found faces in 2D and 3D.
   */
  vector<Box2D3D> detectAllFacesDisparity(const cv::Mat &image, double threshold, const cv::Mat &disparity_image, image_geometry::StereoCameraModel *cam_model);

  /*!
   * \brief Detect all faces in an image and depth image.
   *
   * Input:
   * \param image The image in which to detect faces.
   * \param haar_classifier_filename Path to the xml file containing the trained haar classifier cascade.
   * \param threshold Detection threshold. Currently unused.
   * \param depth_image Image of depth (e.g. from an RGBD camera).
   * \param cam_model The camera model used to convert 2D points to 3D points.
   * Output:
   * A vector of Box2D3Ds containing the bounding boxes around found faces in 2D and 3D.
   */
  vector<Box2D3D> detectAllFacesDepth(const cv::Mat &image, double threshold, const cv::Mat &depth_image, image_geometry::StereoCameraModel *cam_model);

  /*!
   * \brief Initialize the face detector with images and disparities.
   *
   * Initialize the face detector, including loading in the classifier cascade.
   * Input:
   * \param num_cascades Should always be 1 (may change in the future.)
   * \param haar_classifier_filename Full path to the cascade file.
   */
  void initFaceDetectionDisparity(uint num_cascades, string haar_classifier_filename, double face_size_min_m, double face_size_max_m, double max_face_z_m, double face_sep_dist_m);


  /*!
   * \brief Initialize the face detector with images and depth.
   *
   * Initialize the face detector, including loading in the classifier cascade.
   * Input:
   * \param num_cascades Should always be 1 (may change in the future.)
   * \param haar_classifier_filename Full path to the cascade file.
   */
  void initFaceDetectionDepth(uint num_cascades, string haar_classifier_filename, double face_size_min_m, double face_size_max_m, double max_face_z_m, double face_sep_dist_m);

////////////////////
private:

  vector<Face> list_;  /**< The list of face ids. */
  vector<Box2D3D> faces_; /**< The list of face positions. */

  cv::Mat cv_image_gray_;  /**< Grayscale image */
  cv::Mat const* disparity_image_; /**< Disparity image */
  cv::Mat const* depth_image_; /**< Depth image */
  image_geometry::StereoCameraModel *cam_model_; /**< The stereo camera model for 2D-->3D conversions. */

  boost::mutex face_mutex_, face_done_mutex_, t_mutex_;
  boost::mutex* face_go_mutex_;
  boost::thread_group threads_;
  boost::condition face_detection_ready_cond_, face_detection_done_cond_;
  int num_threads_to_wait_for_;
  int images_ready_;

  /* Structures for the face detector. */
  cv::CascadeClassifier cascade_;  /**< Classifier cascade for face detection. */

  void faceDetectionThreadDisparity(uint i);
  void faceDetectionThreadDepth(uint i);

};

};

#endif

