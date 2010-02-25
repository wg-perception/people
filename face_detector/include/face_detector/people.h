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
@verbatim
$ people/bin/face_detection [node_name] [cascade_file_path] [do_display]
@endverbatim

@param node_name A name for the node and topic to publish. Necessary because multiple copies of this node may be run.
@param casade_file_path Full path to the trained haar cascade. Currently useful cascades are people/cascades/haar_frontalface_alt.xml and people/cascades/haar_profileface.xml 
@param do_display 0/1 whether to display the detections or just publish them.

@verbatim
$ people/bin/stereo_face_color_tracker
@endverbatim

@verbatim
$ people/bin/track_starter_gui
@endverbatim

@todo Incorporate color calibration into each node in a nicer way. Currently you have to turn it on in the code.
@todo Correctly interact with the people tracking filter. 
 **/



#include <stdio.h>
#include <iostream>
#include <vector>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>

#include "CvStereoCamModel.h"
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
// Thresholds for the face color tracking algorithm.
#define FACE_SIZE_DEFAULT_M 0.08//0.1
#define COLOR_TRACK_MAX_ITERS 10
#define COLOR_TRACK_EPS_M 0.01
// Timeouts
#define TRACKING_FILTER_TIMEOUT_S 5.0

using namespace std;

/**
   A structure containing the data related to one person.
 */
struct Person {
  CvHistogram *face_color_hist;
  CvHistogram *shirt_color_hist;
  double body_height_3d;
  double body_width_3d;
  CvRect body_bbox_2d;
  IplImage *body_mask_2d;
  double face_size_3d;
  CvRect face_bbox_2d;
  IplImage *face_mask_2d;
  CvScalar face_center_3d;
  string id;
  string name;
  ros::Time last_tracking_filter_update_time;
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

  void freePerson(int iperson);

  /*!
   * \brief Add a person to the list of people.
   */
  void addPerson();

  /*!
   * \brief Return the number of people.
   */
  int getNumPeople();


  /*!
   * \brief Set the tracking filter update time for a given person.
   */
  void setTrackingFilterUpdateTime(ros::Time time, int iperson);

  /*!
   * \brief Return true if the time lapse between the last filter update for a given person and the given time is less than a set threshold.
   */
  bool isWithinTrackingFilterUpdateTimeThresh(ros::Time time, int iperson);

  void killIfFilterUpdateTimeout(ros::Time time);

  /*!
   * \brief Return the face size in 3D
   */
  double getFaceSize3D(int iperson);

  /*!
   * \brief Set a person's 3D face size. For a size of <=0.0, use FACE_SIZE_DEFAULT_M.
   */
  void setFaceSize3D(double face_size, int iperson);

  /*!
   * \brief Set a person's face bbox.
   */
  void setFaceBbox2D(CvRect face_rect, int iperson );

  /*!
   * \brief Print a person's 3d face center position.
   */
  void printFaceCenter3D(int iperson);

  /*!
   * \brief Set a person's 3d face center position.
   */
  void setFaceCenter3D(double cx, double cy, double cz, int iperson);
  
  /*!
   * \brief Find the closest person within a certain face distance in 3D.
   */
  int findPersonFaceLTDist3D(double dist, double cx, double cy, double cz);

  /*!
   * \brief Takes in a rectangle center and size and outputs the four corners in the order (TL; TR; BL; BR)
   */
  void centerSizeToFourCorners( CvMat *centers, CvMat *sizes, CvMat *four_corners);

  /*!
   * \brief Remove a person from the list of people.
   */
  void removePerson(){}

  /*!
   * \brief Use the list of people to recognize a face image.
   */
  void recognizePerson(){}

  /*! 
   * \brief Set a person's id
   */
  void setID(std::string id, int iperson);

  /*!
   * \brief Get a person's id
   */
  std::string getID(int iperson);
  
  /*!
   * \brief Find the first person with a given id.
   */
  int findID(std::string id);

  /*!
   * \brief Make a histogram into an image.
   */
  IplImage* faceHist2Im(int iperson);

  /*!
   * \brief Clear the face color histogram of a given person.
   */
  void clearFaceColorHist(int iperson);


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
  vector<Box2D3D> detectAllFaces(IplImage *image, double threshold, IplImage *disparity_image, CvStereoCamModel *cam_model);
  void initFaceDetection(uint num_cascades, std::vector<string> haar_classifier_filenames);

  // Detect only known faces in an image.
  void detectKnownFaces(){}

  // Track a face.
  void track(){}

  /*!
   * \brief Track a face based on the face colour histogram.
   */
  bool track_color_3d_bhattacharya(const IplImage *image, IplImage *disparity_image, CvStereoCamModel *cam_model, double kernel_radius_m, int npeople,  int* which_people, CvMat* start_points, CvMat* end_points, bool* tracked_each);

 ////////////////////
 private:


  /**< The list of people. */
  vector<Person> list_;

  /**< Grayscale image (to avoid reallocating an image each time an OpenCV function is run.) */
  IplImage *cv_image_gray_;
  IplImage *disparity_image_;
  CvStereoCamModel *cam_model_;
  
  boost::mutex face_mutex_,face_go_mutex_, face_done_mutex_, t_mutex_;
  vector<boost::mutex*> face_go_mutices_;
  boost::thread_group threads_;
  vector<Box2D3D> faces_;
  boost::condition face_detection_ready_cond_, face_detection_done_cond_;
  int num_threads_to_wait_for_;
  int images_ready_;

  /* Structures for the face detector. */
  /**< Classifier cascade for face detection. */
  vector<CvHaarClassifierCascade*>cascades_;
  /**< Storage for OpenCV functions. */
  vector<CvMemStorage*>storages_;

  /* Structures for the color face tracker (cft).*/
  /**< Color planes and normalized color planes. */
  IplImage *cft_r_plane_;
  IplImage *cft_g_plane_;
  IplImage *cft_b_plane_;
  IplImage *cft_r_plane_norm_;
  IplImage *cft_g_plane_norm_;
  IplImage *cft_b_plane_norm_;
  CvMat *rbins_;
  CvMat *gbins_;
  /**< The 3d coords for each point. */
  IplImage *cft_X_;
  IplImage *cft_Y_;
  IplImage *cft_Z_;
  CvMat *cft_xyz_;
  CvMat *cft_uvd_;
  CvHistogram *cft_start_hist_;
  CvHistogram *cft_ratio_hist_;

  void faceDetectionThread(uint i);

};

#endif

