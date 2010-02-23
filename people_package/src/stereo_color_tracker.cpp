/*********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc
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

#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "ros/node.h" //TODO: get rid of this
#include "ros/console.h"
#include "CvStereoCamModel.h"
#include "color_calib.h"
#include <people/PositionMeasurement.h>
#include "stereo_msgs/StereoInfo.h"
#include "stereo_msgs/DisparityInfo.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "people/ColoredLines.h"
#include "topic_synchronizer/topic_synchronizer.h"
#include "tf/transform_listener.h"
#include <tf/message_notifier.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <boost/thread/mutex.hpp>

#include "utils.h"

namespace people
{

  bool DEBUG_DISPLAY = false;

  using namespace std;

  // StereoColorTracker - Color histogram-based tracking from stereo cameras.

  class StereoColorTracker{
  public:
    // ROS
    ros::NodeHandle nh_;

    // Images and conversion
    sensor_msgs::Image limage_;
    sensor_msgs::Image dimage_;
    stereo_msgs::StereoInfo stinfo_;
    stereo_msgs::DisparityInfo dispinfo_;
    sensor_msgs::CameraInfo rcinfo_;
    sensor_msgs::CvBridge lbridge_;
    sensor_msgs::CvBridge dbridge_;
    color_calib::Calibration *lcolor_cal_;
    TopicSynchronizer<StereoColorTracker> *sync_;

    bool on_robot_; // If true, use tf. Otherwise, ignore the frame.
    tf::TransformListener *tf_;
    message_filters::Subscriber<people::PositionMeasurement>* person_sub_;
    tf::MessageFilter<people::PositionMeasurement>* tf_person_filter_;
    //tf::MessageNotifier<people::PositionMeasurement>* message_notifier_person_;
    ros::Subscriber person_direct_sub_;

    ros::Publisher lines_pub_;
    ros::Publisher position_pub_;

    string fixed_frame_;

    ros::Time last_image_time_;

    bool use_depth_;
  
    bool calib_color_;
    bool do_display_;
    bool adapt_hist_;

    double kernel_size_m_;
    double object_radius_m_;

    people::PositionMeasurement pos_;
    struct RestampedPositionMeasurement {
      ros::Time restamp;
      people::PositionMeasurement pos;
      CvHistogram *hist;
    };
    map<string, RestampedPositionMeasurement> current_pos_list_; /**< Queue of updated people positions from the filter. */


    bool quit_;

    boost::mutex pos_mutex_;

    /////////////////////////////////////////////////////////////////////
    StereoColorTracker() :
      last_image_time_(ros::Time().fromSec(0)),
      use_depth_(true),
      quit_(false),
      cv_image_left_(NULL),
      cv_image_disp_(NULL),
      cam_model_(NULL),
      initialized_(false),
      uvd_(NULL),
      xyz_(NULL),
      XYZ_(NULL),
      r_plane_norm_(NULL),
      g_plane_norm_(NULL),
      b_plane_norm_(NULL),
      r_bins_mat_(NULL),
      g_bins_mat_(NULL),
      last_blob_num_(0)
    { 

      if (DEBUG_DISPLAY) {
	cvNamedWindow("Current Hist", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("New Hist", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Backprojection",CV_WINDOW_AUTOSIZE);
      } 

      nh_.param("/people/color_tracker/on_robot", on_robot_, true);
      nh_.param("/people/color_tracker/fixed_frame", fixed_frame_, std::string("map"));
      nh_.param("/people/color_tracker/display", do_display_, true);
      nh_.param("/people/color_tracker/adapt_hist", adapt_hist_, false);
      nh_.param("/people/color_tracker/calib_color", calib_color_, false);
      nh_.param("/people/color_tracker/object_radius_m", object_radius_m_, 0.2);
      double kernel_obj_size_ratio;
      nh_.param("/people/color_tracker/kernel_to_object_size_ratio", kernel_obj_size_ratio, 2.0);
      kernel_size_m_ = object_radius_m_ * kernel_obj_size_ratio;

      ROS_INFO_STREAM_NAMED("color_tracker","Kernel size (m): " << kernel_size_m_);

      calib_color_ = false;

      lcolor_cal_ = new color_calib::Calibration(nh_);
      sync_ = new TopicSynchronizer<StereoColorTracker>(nh_.getNode(), this, &StereoColorTracker::imageCBAll, ros::Duration().fromSec(0.05), &StereoColorTracker::imageCBTimeout);

      // Advertise a 3d position measurement for each head.
      position_pub_=nh_.advertise<people::PositionMeasurement>("people_tracker_measurements",1);

      if (do_display_) {
	lines_pub_ = nh_.advertise<people::ColoredLines>("lines_to_draw",1);
	ROS_INFO_STREAM_NAMED("color_tracker","Advertising colored lines to draw remotely.");
      }

      // Subscribe to the images and parameters
      sync_->subscribe("stereo/left/image_rect_color",limage_,1);
      sync_->subscribe("stereo/disparity",dimage_,1);
      sync_->subscribe("stereo/stereo_info",stinfo_,1);
      sync_->subscribe("stereo/disparity_info",dispinfo_,1);
      sync_->subscribe("stereo/right/cam_info",rcinfo_,1);
      sync_->ready();

      // Subscribe to the tracker measurements to initialize your position.
      if (on_robot_) {
	tf_ = new tf::TransformListener(); 
	person_sub_ = new message_filters::Subscriber<people::PositionMeasurement>(nh_, "people_tracker_measurements", 1);
	tf_person_filter_ = new tf::MessageFilter<people::PositionMeasurement>(*person_sub_, *tf_, fixed_frame_, 10);
	tf_person_filter_->registerCallback(boost::bind(&StereoColorTracker::initPosCB, this, _1));
      }
      else {
	person_direct_sub_ = nh_.subscribe("people_tracker_measurements",1,&StereoColorTracker::initPosCB2, this);
      }
    }
  
    /////////////////////////////////////////////////////////////////////
    ~StereoColorTracker()
    {
    
      cvReleaseImage(&r_plane_norm_); r_plane_norm_ = 0;
      cvReleaseImage(&g_plane_norm_); g_plane_norm_ = 0;
      cvReleaseImage(&b_plane_norm_); b_plane_norm_ = 0;
      cvReleaseMat(&uvd_); uvd_ = 0;
      cvReleaseMat(&xyz_); xyz_ = 0;
      cvReleaseMat(&XYZ_); XYZ_ = 0;
      cvReleaseImage(&r_bins_mat_); r_bins_mat_=0;
      cvReleaseImage(&g_bins_mat_); g_bins_mat_=0;
      
      map<string, RestampedPositionMeasurement>::iterator it;
      for (it = current_pos_list_.begin(); it != current_pos_list_.end(); it++) {
	cvReleaseHist(&((*it).second.hist)); (*it).second.hist = 0;
      }

      delete cam_model_;
    }

    /////////////////////////////////////////////////////////////////////
    // Initialize the person's position.
    void initPosCB(const tf::MessageNotifier<const people::PositionMeasurement>::MessagePtr& person_pos_msg)
    {
      boost::mutex::scoped_lock pos_lock(pos_mutex_);

      if (person_pos_msg->initialization) {
	if (pos_.pos.z == 0.0) {
	  ROS_WARN("The initial position has an invalid z-value. Please choose another point.");
	}
	else {

	  map<string, RestampedPositionMeasurement>::iterator it;
	  std::string person_pos_msg__object_id = person_pos_msg->object_id;
	  if (person_pos_msg__object_id == "") {
	    stringstream ss;
	    ss << "sct_" << last_blob_num_;
	    last_blob_num_++;
	    person_pos_msg__object_id = ss.str();
	    it = current_pos_list_.end();
	  }
	  else {
	    it = current_pos_list_.find(person_pos_msg__object_id);
	  }
	  RestampedPositionMeasurement rpm;
	  rpm.pos = *person_pos_msg;
	  rpm.restamp = person_pos_msg->header.stamp;
	  rpm.hist = 0;
	  if (it == current_pos_list_.end()) {
	    current_pos_list_.insert(pair<string, RestampedPositionMeasurement>(person_pos_msg__object_id, rpm));
	  }
	  else {
	    cvReleaseHist(&((*it).second.hist)); (*it).second.hist = 0;
	    (*it).second = rpm;
	  }
	  initialized_ = true;
	  ROS_INFO_STREAM_NAMED("color_tracker","Track for person " << person_pos_msg__object_id << " initialized.");
	}
      }
    }   


    /////////////////////////////////////////////////////////////////////
    // Initialize the person's position.
    void initPosCB2(const people::PositionMeasurement::ConstPtr& person_pos_msg)
    {
      boost::mutex::scoped_lock pos_lock(pos_mutex_);

      pos_ = *person_pos_msg;

      if (pos_.initialization) {
	if (pos_.pos.z == 0.0) {
	  ROS_WARN("The initial position has an invalid z-value. Please choose another point.");
	}
	else {

	  map<string, RestampedPositionMeasurement>::iterator it;

	  if (pos_.object_id == "") {
	    stringstream ss;
	    ss << "sct_" << last_blob_num_;
	    last_blob_num_++;
	    pos_.object_id = ss.str();
	    it = current_pos_list_.end();
	  }
	  else {
	    it = current_pos_list_.find(pos_.object_id);
	  }
	  RestampedPositionMeasurement rpm;
	  rpm.pos = pos_;
	  rpm.restamp = pos_.header.stamp;
	  rpm.hist = NULL;
	  if (it == current_pos_list_.end()) {
	    current_pos_list_.insert(pair<string, RestampedPositionMeasurement>(pos_.object_id, rpm));
	  }
	  else {
	    cvReleaseHist(&((*it).second.hist)); (*it).second.hist = 0;
	    (*it).second = rpm;
	  }
	  initialized_ = true;
	  ROS_INFO_STREAM_NAMED("color_tracker","Track for person " << pos_.object_id << " initialized.");
	}
      }
    }

    /////////////////////////////////////////////////////////////////////
    // The image callback when not all topics are sync'ed. Don't do anything, just wait for sync.
    void imageCBTimeout(ros::Time t) {
      ROS_DEBUG_STREAM_NAMED("color_tracker","Message timeout");
    }


    /////////////////////////////////////////////////////////////////////
    void imageCBAll(ros::Time t)
    {
      if (!initialized_) 
	return;

      // Timing
      struct timeval timeofday;
      gettimeofday(&timeofday,NULL);
      ros::Time startt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);

      last_image_time_ = limage_.header.stamp;

      // Convert images to OpenCV.
      if (lbridge_.fromImage(limage_,"bgr8")) {
	cv_image_left_ = lbridge_.toIpl();
	cvSmooth(cv_image_left_, cv_image_left_, CV_GAUSSIAN, 5);
	if (calib_color_) {
	  if ( lcolor_cal_->getFromParam("stereo/left/image_rect_color")) {
	    // Color calibration.
	    lcolor_cal_->correctColor(cv_image_left_, cv_image_left_, true, true, COLOR_CAL_BGR);
	  }
	  else {
	    // Exit if color calibration hasn't been performed.
	    ROS_DEBUG_STREAM_NAMED("color_tracker","Color calibration not available.");
	    return;
	  }
	}
      }

      // Set the disparity image.
      if (dbridge_.fromImage(dimage_)) {
	cv_image_disp_ = dbridge_.toIpl();
      }
      else {
	return;
      }

      // Convert the stereo calibration into a camera model. Only done once.
      if (!cam_model_) {
	double Fx = rcinfo_.P[0];
	double Fy = rcinfo_.P[5];
	double Clx = rcinfo_.P[2];
	double Crx = Clx;
	double Cy = rcinfo_.P[6];
	double Tx = -rcinfo_.P[3]/Fx;
	cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,1.0/dispinfo_.dpp);
      }
      cout << "cam model" << endl;

      // Transform the last known position to the current time.
      boost::mutex::scoped_lock pos_lock(pos_mutex_);

      map<string, RestampedPositionMeasurement>::iterator it;
      if (on_robot_) {  
	for (it = current_pos_list_.begin(); it != current_pos_list_.end(); it++) {
	  tf::Point pt;
	  tf::pointMsgToTF((*it).second.pos.pos, pt);
	  tf::Stamped<tf::Point> loc(pt, (*it).second.pos.header.stamp, (*it).second.pos.header.frame_id);
	  try
	    {
	      tf_->transformPoint(limage_.header.frame_id, last_image_time_, loc, fixed_frame_, loc); 
	    } 
	  catch (tf::TransformException& ex)
	    {
	      ROS_DEBUG_STREAM_NAMED("color_tracker","TF exception " << ex.what());
	    }   
	  (*it).second.pos.header.stamp = limage_.header.stamp;
	  (*it).second.pos.pos.x = loc[0];
	  (*it).second.pos.pos.y = loc[1];
	  (*it).second.pos.pos.z = loc[2];
	}
      }
      else {
	for (it = current_pos_list_.begin(); it != current_pos_list_.end(); it++) {
	  (*it).second.pos.header.stamp = limage_.header.stamp;
	}
      }

      // Image size
      CvSize im_size = cvGetSize(cv_image_left_);

      // If not allocated, create the color planes and 3d pos arrays.
      if (r_bins_mat_==NULL) {
	uvd_ = cvCreateMat(im_size.width*im_size.height,3,CV_32FC1);
	xyz_ = cvCreateMat(im_size.width*im_size.height,3,CV_32FC1);
	XYZ_ = cvCreateMatHeader(im_size.height, im_size.width, CV_32FC1);
	cvReshape(xyz_, XYZ_, 3, im_size.height);
	r_bins_mat_ = cvCreateImage(im_size, IPL_DEPTH_8U, 1);
	g_bins_mat_ = cvCreateImage(im_size, IPL_DEPTH_8U, 1);
      }
      
     
      ////////////////////////////////
      // Get a 3d point for each pixel

      float *fptr = (float*)(uvd_->data.ptr);
      ushort *cptr;
      for (int v =0; v < im_size.height; ++v) {
	cptr = (ushort*)(cv_image_disp_->imageData+v*cv_image_disp_->widthStep);
 	for (int u=0; u<im_size.width; ++u) {
 	  (*fptr) = (float)u; ++fptr;
 	  (*fptr) = (float)v; ++fptr;
 	  (*fptr) = (float)(*cptr); ++fptr; ++cptr;      
 	}
       }
       cam_model_->dispToCart(uvd_,xyz_);   // What happens to invalid disparity values??? d=0, z=0      

      ///////////////////////////////////


      // Possible change: equalize each image channel.
      // cvCvtPixToPlane(cv_image_left_, b_plane_norm_, g_plane_norm_, r_plane_norm_, 0);
      // cvEqualizeHist( cft_r_plane_, cft_r_plane_norm_);

      // Compute the r and g histogram bin for each pixel. Will take the floor of each val since the bin arrays are unsigned shorts.
      uchar *rptr, *gptr, *rbptr, *gbptr;
      for (int v=0; v < im_size.height; ++v) {
	rptr = (uchar*)(cv_image_left_->imageData + v*cv_image_left_->widthStep);
	rbptr = (uchar*)(r_bins_mat_->imageData+v*r_bins_mat_->widthStep);
	gbptr = (uchar*)(g_bins_mat_->imageData+v*g_bins_mat_->widthStep);
	for (int u=0; u < im_size.width; ++u) {
	  ++rptr; //Skip b.
	  (*gbptr) = (uchar) ( floor( ((double)(*rptr))/2.0 ) ); ++rptr;
	  (*rbptr) = (uchar) ( floor( ((double)(*rptr))/2.0 ) ); ++rptr; 
	  ++rbptr; ++gbptr;
	}
      }
    
      RestampedPositionMeasurement *current_pos;
      for (it = current_pos_list_.begin(); it != current_pos_list_.end(); it++) {
	current_pos = &((*it).second);

	// Get the 2d point and bbox of the current location.
	// (tl,tr,bl,br)
	float txyz[3] = {current_pos->pos.pos.x, current_pos->pos.pos.y, current_pos->pos.pos.z};
	CvMat *centerxyz = cvCreateMatHeader(1,3,CV_32FC1);
	cvInitMatHeader(centerxyz,1,3,CV_32FC1,txyz);
	CvMat *centeruvd = cvCreateMat(1,3,CV_32FC1);
	CvMat *uvds = cvCreateMat(2,3,CV_32FC1);
	centerSize3DToLTRB2D(centerxyz,object_radius_m_,uvds);
	cam_model_->cartToDisp(centerxyz, centeruvd);
	
	// Compute the histogram at the current location.
	int hsizes[2] = {128,128};
	CvHistogram *new_hist = cvCreateHist(2, hsizes, CV_HIST_ARRAY);

	int num_points_in_kernel = computeRGHist(uvds, centerxyz, im_size, new_hist);
	// If there are no points in this kernel, this is an invalid position.
	if (num_points_in_kernel == 0) {
	  ROS_DEBUG_STREAM_NAMED("color_tracker","No points in kernel" << cvmGet(centerxyz,0,0) << " " << cvmGet(centerxyz,0,1) << " " << cvmGet(centerxyz,0,2));
	  cvReleaseMat(&centerxyz); centerxyz=0;
	  cvReleaseMat(&centeruvd); centeruvd=0;
	  cvReleaseMat(&uvds); uvds=0;
	  continue;
	}

	// If this is the first frame, set the histogram and continue to the next blob.
	if (current_pos->hist == NULL) {
	  current_pos->hist = new_hist;
	  publishPoint(centerxyz,uvds,&(current_pos->pos));
	  cvReleaseMat(&centerxyz); centerxyz=0;
	  cvReleaseMat(&centeruvd); centeruvd=0;
	  cvReleaseMat(&uvds); uvds=0;
	
	  cout << "first frame " << endl;
	  // Timing
	  gettimeofday(&timeofday,NULL);
	  ros::Time endt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
	  ros::Duration diff = endt-startt;
	  ROS_DEBUG_STREAM_NAMED("color_tracker","Start " << startt.toSec() << " End " << endt.toSec() << " Duration " << diff.toSec() );
	  
	  continue;
	}   

	CvHistogram *current_hist = current_pos->hist;

	//// Debug display
	if (DEBUG_DISPLAY) {
	  showHistogram(current_hist,"Current Hist");
	  showHistogram(new_hist,"New Hist");

	  IplImage *hist_ratio = cvCreateImage(im_size, IPL_DEPTH_32F, 1);
	  float tmax = 0.0;
	  for (int v=0; v<=im_size.height; ++v) {
	    rptr = (uchar*)(r_bins_mat_->imageData + v*r_bins_mat_->widthStep);
	    gptr = (uchar*)(g_bins_mat_->imageData + v*g_bins_mat_->widthStep);
	    fptr = (float*)(hist_ratio->imageData + v*hist_ratio->widthStep);
	    for (int u = 0; u<=im_size.width; ++u) {
	      (*fptr) = sqrt(cvQueryHistValue_2D(current_hist, *rptr, *gptr)/cvQueryHistValue_2D(new_hist, *rptr, *gptr));
	      if ((*fptr)/1000.0>tmax) {
		tmax = (*fptr)/1000.0;
	      }
	      rptr++; gptr++; fptr++;
	    }
	  }
      
	  for (int v=0; v<=im_size.height; ++v) {
	    fptr = (float*)(hist_ratio->imageData + v*hist_ratio->widthStep);
	    for (int u = 0; u<=im_size.width; ++u) {
	      (*fptr) = (*fptr)/tmax;
	      fptr++;
	    }
	  }

	  cout << "Max ratio val " << tmax << endl;

	  cvShowImage("Backprojection",hist_ratio);
	  cvWaitKey(7);

	  cvReleaseImage(&hist_ratio); hist_ratio = 0;
	}
	////

	// Find the blob in the current frame

      
	/*** Mean shift iterations. ***/
	double EPS = 0.01*0.01; // Squared!!!
	CvMat *curr_point = cvCloneMat(centerxyz);
	CvMat *next_point = cvCloneMat(centerxyz);
	cvSetZero(next_point);
	double bhat_coeff, bhat_coeff_new;
	double total_weight = 0.0;
	num_points_in_kernel = 0;
	double wi = 0.0;
	float cp[3];
	int u1, u2, v1, v2;
	float dx, dy, dz, d;
	float *xptr, *nptr;
	double denom = kernel_size_m_*kernel_size_m_;

	// Compute the Bhattacharya coeff of the start hist vs the true face hist.
	bhat_coeff = cvCompareHist( current_hist, new_hist, CV_COMP_BHATTACHARYYA);
      
	int iter;
	for (iter = 0; iter < 10; iter++) {

	  // Compute the next location of the center
	  d = 0.0; // d = total distance moved (squared)
	  total_weight = 0.0;
	  num_points_in_kernel = 0;
	  wi = 0.0;
	  cvSetZero(next_point); 
	  fptr = (float*)(curr_point->data.ptr);
	  cp[0] = (*fptr); fptr++;
	  cp[1] = (*fptr); fptr++;
	  cp[2] = (*fptr); fptr++; 
	  fptr = NULL;
	
	  // For each pixel within the kernel distance of the current position...
	  if (cp[2]==0.0) {
	    // The current point should never have a z-val of 0. 
	    ROS_DEBUG_STREAM_NAMED("color_tracker","Current point has z=0");
	    break;
	}

	  centerSize3DToLTRB2D(curr_point, kernel_size_m_, uvds);
	  
	  u1 = MAX((int)cvmGet(uvds,0,0),0);
	  u2 = MIN((int)cvmGet(uvds,1,0),im_size.width);
	  v1 = MAX((int)cvmGet(uvds,0,1),0);
	  v2 = MIN((int)cvmGet(uvds,1,1),im_size.height);

	  for (int v=v1; v<=v2; ++v) {
	    xptr = (float*)(XYZ_->data.ptr + v*XYZ_->step) + 3*u1;
	    rptr = (uchar*)(r_bins_mat_->imageData + v*r_bins_mat_->widthStep) + u1;
	    gptr = (uchar*)(g_bins_mat_->imageData + v*g_bins_mat_->widthStep) + u1;
	    for (int u = u1; u<=u2; ++u) {
	      if (*(xptr+2) != 0.0) {
		dx = (*xptr)-cp[0];
		dy = (*(xptr+1))-cp[1];
		dz = (*(xptr+2))-cp[2]; 
		d = (dx*dx + dy*dy + dz*dz)/denom;
		if (d <= 1.0){
		  wi = sqrt(cvQueryHistValue_2D(current_hist, *rptr, *gptr)/cvQueryHistValue_2D(new_hist, *rptr, *gptr));
		  total_weight += wi;
		  nptr = (float *)(next_point->data.ptr);
		  (*nptr) += wi*dx; ++nptr;
		  (*nptr) += wi*dy; ++nptr;  
		  (*nptr) += wi*dz;
		  num_points_in_kernel++;
		}
	      }	  

	      xptr+=3;
	      ++rptr; ++gptr;
	    }
	  }

	  // If there are no points in this kernel, don't move the point.
	  if (num_points_in_kernel == 0) {
	    ROS_DEBUG_STREAM_NAMED("color_tracker","No points in kernel" << cvmGet(curr_point,0,0) << " " << cvmGet(curr_point,0,1) << " " << cvmGet(curr_point,0,2));
	    break;
	  }
	  // Move the point
	  d = 0.0;      
	  nptr = (float *)(next_point->data.ptr);
	  for (int i=0; i<3; i++) {
	    (*nptr) /= total_weight;
	    d += (*nptr)*(*nptr);
	    (*nptr) += cp[i];
	    nptr++;
	  } 
	
	  // This next loop backtracks the mean shift move if the new Bhattacharyya coeff is worse than the old one.
	  // This is necessary because the move above isn't guaranteed to give a higher coeff value.
	  bhat_coeff_new = 0.0;
	  
	  // Compute the histogram and Bhattacharya coeff at the new position.
	  // Convert 3d point-size to 2d rectangle
	  centerSize3DToLTRB2D(next_point, object_radius_m_, uvds); 
	  num_points_in_kernel = computeRGHist(uvds, next_point, im_size, new_hist);      
	  // If there are no points in this kernel, this is an invalid position.
	  if (num_points_in_kernel == 0) {
	    ROS_DEBUG_STREAM_NAMED("color_tracker","No points in kernel" << cvmGet(centerxyz,0,0) << " " << cvmGet(centerxyz,0,1) << " " << cvmGet(centerxyz,0,2));
	    break;
	  }
	  bhat_coeff_new = cvCompareHist( current_hist, new_hist, CV_COMP_BHATTACHARYYA); 
	
	  while (true) {
	    // If you've moved far but the Bhattacharyya coeff is worse, backtrack half way.
	    if ((d <= EPS) || (bhat_coeff <= bhat_coeff_new)) {
	      break;
	    }

	    ROS_DEBUG_STREAM_NAMED("color_tracker","Backtracking");
	    d = 0.0;
	    fptr = (float*)(next_point->data.ptr);
	    for (int i=0; i<3; i++) {
	      (*fptr) += cp[i];
	      (*fptr) /= 2.0;
	      d += ((*fptr)-cp[i])*((*fptr)-cp[i]);
	      fptr++;
	    }
	    fptr = NULL;
	  
	    // Compute the histogram and Bhattacharya coeff at the new position.
	    centerSize3DToLTRB2D(next_point, object_radius_m_, uvds); 
	    num_points_in_kernel = computeRGHist(uvds, next_point, im_size, new_hist);      
	    // If there are no points in this kernel, this is an invalid position. Set the new coeff low so that the backtracking continues. 
	    if (num_points_in_kernel == 0) {
	      ROS_DEBUG_STREAM_NAMED("color_tracker","No points in kernel" << cvmGet(next_point,0,0) << " " << cvmGet(next_point,0,1) << " " << cvmGet(next_point,0,2));
	      bhat_coeff_new = -1.0;
	    }
	    else {
	      bhat_coeff_new = cvCompareHist( current_hist, new_hist, CV_COMP_BHATTACHARYYA); 
	    }
	    
	  }

	  cvCopy(next_point,curr_point);
	  cvSetZero(next_point);
	  bhat_coeff = bhat_coeff_new;
	    

	  // Movement is small, break.
	  if (d <= EPS) {
	    break;
	  }

	  // new_hist was already updated in the loop above.
	  
	}
      
	// Adapt the histogram, if requested. This could cause drift.
	if (adapt_hist_) {
	  current_hist = new_hist;
	  new_hist = 0;
	}

	// Publish the 3d head center for this person and set it for the next callback.
	centerSize3DToLTRB2D(curr_point,object_radius_m_, uvds);
	// If the new histogram is good, publish the point. Otherwise, clear the point and stop tracking.
	if (bhat_coeff > 0.3) {
	  publishPoint(curr_point, uvds, &(current_pos->pos));
	}
	else {
	  ROS_INFO_STREAM_NAMED("color_tracker","\n\nLost target\n\n");
	  initialized_ = false;
	}

	ROS_DEBUG_STREAM_NAMED("color_tracker","Bhat coeff final: " << bhat_coeff << ", Pos " << cvmGet(curr_point,0,0) << ", " << cvmGet(curr_point,0,1) << ", " << cvmGet(curr_point,0,2) );
 
	
	// Cleanup
	cvReleaseMat(&centerxyz); centerxyz = 0;
	cvReleaseMat(&centeruvd); centeruvd = 0;
	cvReleaseMat(&uvds); uvds = 0;
	cvReleaseMat(&curr_point); curr_point = 0;
	cvReleaseMat(&next_point); next_point = 0;
	cvReleaseHist(&new_hist); new_hist = 0;
	current_hist = 0;


      } // End tracking this blob.


      // Timing
      cout << "endtime" << endl;
      gettimeofday(&timeofday,NULL);
      ros::Time endt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
      ros::Duration diff = endt-startt;
      ROS_DEBUG_STREAM_NAMED("color_tracker","Duration " << diff.toSec() );
    }


  private:

    // The left and disparity images.
    IplImage *cv_image_left_;
    IplImage *cv_image_disp_;

    CvStereoCamModel *cam_model_;
  
    bool initialized_;

    CvMat *uvd_, *xyz_;
    CvMat *XYZ_;
    IplImage *r_plane_norm_, *g_plane_norm_, *b_plane_norm_;
    IplImage *r_bins_mat_, *g_bins_mat_;

    int last_blob_num_;


    /////////////////////////////////////////////////////////////////////
    // Publish the 3D center point of the object and an optional 2D box for visualization.
    void publishPoint(CvMat* curr_point, CvMat *uvds, people::PositionMeasurement *pos ) {
      // Publish the 3d head center for this person.

      // Update the restamped point.
      // frame_id and stamp should be ok from the transformation.
      pos->name = "stereo_color_tracker";
      pos->pos.x = cvmGet(curr_point,0,0);
      pos->pos.y = cvmGet(curr_point,0,1);
      pos->pos.z = cvmGet(curr_point,0,2);
      pos->reliability = 0.5;
      pos->initialization = 0;
      pos->covariance[0] = 0.09; pos->covariance[1] = 0.0;  pos->covariance[2] = 0.0;
      pos->covariance[3] = 0.0;  pos->covariance[4] = 0.09; pos->covariance[5] = 0.0;
      pos->covariance[6] = 0.0;  pos->covariance[7] = 0.0;  pos->covariance[8] = 0.09;


      position_pub_.publish(*pos);//people_tracker_measurements


      if (do_display_) {
	people::ColoredLines all_cls;
	vector<people::ColoredLine> lines; 
	lines.resize(4); 
	lines[0].r = 255; lines[0].g = 0; lines[0].b = 0;
	lines[1].r = 255; lines[1].g = 0; lines[1].b = 0;
	lines[2].r = 255; lines[2].g = 0; lines[2].b = 0;
	lines[3].r = 255; lines[3].g = 0; lines[3].b = 0;
	// l->r top
	lines[0].x0 = cvmGet(uvds,0,0);
	lines[0].y0 = cvmGet(uvds,0,1);
	lines[0].x1 = cvmGet(uvds,1,0);
	lines[0].y1 = cvmGet(uvds,0,1);
	// t->b left
	lines[1].x0 = cvmGet(uvds,0,0);
	lines[1].y0 = cvmGet(uvds,0,1);
	lines[1].x1 = cvmGet(uvds,0,0);
	lines[1].y1 = cvmGet(uvds,1,1);
	// l->r bottom
	lines[2].x0 = cvmGet(uvds,0,0);
	lines[2].y0 = cvmGet(uvds,1,1);
	lines[2].x1 = cvmGet(uvds,1,0);
	lines[2].y1 = cvmGet(uvds,1,1);
	// t->b right
	lines[3].x0 = cvmGet(uvds,1,0);
	lines[3].y0 = cvmGet(uvds,0,1);
	lines[3].x1 = cvmGet(uvds,1,0);
	lines[3].y1 = cvmGet(uvds,1,1);
	all_cls.header.stamp = limage_.header.stamp;
	all_cls.label = "color_tracker";
	all_cls.header.frame_id = limage_.header.frame_id;
	all_cls.lines = lines;
	lines_pub_.publish(all_cls); //"lines_to_draw"
      }

    }


    /////////////////////////////////////////////////////////////////////
    // Compute the R-G histogram for the points less than kernel_size_m_ from the center, or perform the mean shift iteration.
    int computeRGHist(CvMat *uvds, CvMat *centerxyz, CvSize im_size, CvHistogram *new_hist ) 
    { 
      int num_points_in_kernel = 0;
      int u,v,bbv1,bbv2,bbu1,bbu2;
      double d, dx, dy, dz, cx, cy, cz;
      float *xptr, *binptr;
      uchar *rptr,*gptr;
      cx = cvmGet(centerxyz,0,0);
      cy = cvmGet(centerxyz,0,1);
      cz = cvmGet(centerxyz,0,2);

      // Set the entire histogram to a small number to avoid 0 bins (and division by zero later).
      cvSet(new_hist->bins,cvScalar(0.0000001));

      double band = object_radius_m_; 
      band = band*band;

      // Fix out of bounds issues.
      bbv1 = MAX(cvmGet(uvds,0,1),0);
      bbv2 = MIN(cvmGet(uvds,1,1),im_size.height);
      bbu1 = MAX(cvmGet(uvds,0,0),0);
      bbu2 = MIN(cvmGet(uvds,1,0),im_size.width);

      for (v = bbv1; v < bbv2; ++v) {
	xptr = (float *)(XYZ_->data.ptr + v*XYZ_->step) + 3*bbu1;
	rptr = (uchar *)(r_bins_mat_->imageData + v*r_bins_mat_->widthStep) + bbu1;
	gptr = (uchar *)(g_bins_mat_->imageData + v*g_bins_mat_->widthStep) + bbu1;
	for (u = bbu1; u < bbu2; ++u) {	  
	  if (*(xptr+2) != 0.0) {
	    dx = ((*xptr) - cx);
	    dy = ((*(xptr+1)) - cy);
	    dz = ((*(xptr+2)) - cz);
	    d = (dx*dx+dy*dy+dz*dz)/band;
	    if (d <= 1.0){
	      binptr = (float*)cvPtr2D(new_hist->bins, *rptr, *gptr);
	      (*binptr) += 1-d;
	      //cvSetReal2D(new_hist->bins, *rptr, *gptr, 1-d + cvGetReal2D(new_hist->bins, *rptr, *gptr)); 
	      ++num_points_in_kernel;
	      // Ignore normalization constant since we'll normalize the histogram below.
	    }
	  }
	  else {	
	  }
	  xptr+=3;
	  ++rptr; ++gptr;
	}
      }

      cvNormalizeHist(new_hist,1.0);
      return num_points_in_kernel;

    }

    /////////////////////////////////////////////////////////////////////
    // Get the left-top and right-bottom 2D coordinates for the 3D corner points of the face.
    void centerSize3DToLTRB2D(CvMat *centerxyz, double radius, CvMat *uvds) 
    {

      float cx1 = cvmGet(centerxyz,0,0) - radius; 
      float cx2 = cx1 + 2.0*radius;
      float cy1 = cvmGet(centerxyz,0,1) - radius;
      float cy2 = cy1 + 2.0*radius;
      float cz = cvmGet(centerxyz,0,2);
      float txyzs[] = {
	cx1, cy1, cz,
	cx2, cy2, cz
      };
      CvMat xyzs;
      cvInitMatHeader(&xyzs,2,3,CV_32FC1,&txyzs);
      cam_model_->cartToDisp(&xyzs, uvds);      

    }

    //////////////////////////////////////////////////////////////////////
    // Display a 2D histogram in an OpenCV window
    void showHistogram(CvHistogram *hist, const char* window_name) {
      IplImage *img = cvCreateImage(cvSize(128,128), IPL_DEPTH_8U,1);
      float min, max;
      cvGetMinMaxHistValue(hist, &min, &max);
      for (int i=0; i<128; i++) {
	for (int j=0; j<128; j++) {
	  cvSetReal2D(img,i,j,(round)(255*cvQueryHistValue_2D(hist,i,j)/max));
	}
      }
      cvShowImage(window_name,img);
      cvReleaseImage(&img);
    }


  }; // class
}; // namespace


/////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "stereo_color_tracker");

  people::StereoColorTracker sct();

  ros::spin ();
  return (0);
}
