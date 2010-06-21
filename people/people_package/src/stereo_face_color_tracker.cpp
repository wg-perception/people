/*********************************************************************
 * A ros node to run face detection and colour-based face tracking.
 *
 **********************************************************************
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
#include <people_package/PositionMeasurement.h>
#include "stereo_msgs/StereoInfo.h"
#include "stereo_msgs/DisparityInfo.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "topic_synchronizer/topic_synchronizer.h"
#include "tf/transform_listener.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <boost/thread/mutex.hpp>

#include "people_package/people.h"
#include "utils.h"

#define __FACE_COLOR_TRACKER_DEBUG__ 0
#define __FACE_COLOR_TRACKER_DISPLAY__ 1

using namespace std;

// StereoFaceColorTracker - Color histogram-based tracking from stereo cameras.

class StereoFaceColorTracker: public ros::Node {
public:
  // Images and conversion
  sensor_msgs::Image limage_;
  sensor_msgs::Image dimage_;
  stereo_msgs::StereoInfo stinfo_;
  stereo_msgs::DisparityInfo dispinfo_;
  sensor_msgs::CameraInfo rcinfo_;
  sensor_msgs::CvBridge lbridge_;
  sensor_msgs::CvBridge dbridge_;
  color_calib::Calibration lcolor_cal_;
  TopicSynchronizer<StereoFaceColorTracker> sync_;
  ros::Time last_image_time_;

  // The left and disparity images.
  IplImage *cv_image_left_;
  IplImage *cv_image_disp_;
  IplImage *cv_image_disp_out_;

  bool use_depth_;
  CvStereoCamModel *cam_model_;
  
  bool calib_color_;

  People *people_;
  bool detect_faces_;
  people::PositionMeasurement init_pos_;
  const char *haar_filename_;

  bool quit_;
  int detect_;

  IplImage *X_, *Y_, *Z_;
  CvMat *uvd_, *xyz_;

  tf::TransformListener tf;

  boost::mutex cv_mutex_;

  ros::NodeHandle nh_;

  StereoFaceColorTracker(bool detect_faces, const char *haar_filename, bool use_depth, bool calib_color) : 
    ros::Node("stereo_face_color_tracker",ros::Node::ANONYMOUS_NAME),
    lcolor_cal_(nh_),
    sync_(this, &StereoFaceColorTracker::image_cb_all, ros::Duration().fromSec(0.05), &StereoFaceColorTracker::image_cb_timeout),
    last_image_time_(ros::Time().fromSec(0)),
    cv_image_left_(NULL),
    cv_image_disp_(NULL),
    cv_image_disp_out_(NULL),
    use_depth_(use_depth),
    cam_model_(NULL),
    calib_color_(false),
    people_(NULL),
    detect_faces_(detect_faces),
    haar_filename_(haar_filename),
    quit_(false),
    detect_(0),
    X_(NULL),
    Y_(NULL),
    Z_(NULL),
    uvd_(NULL),
    xyz_(NULL),
    tf()
  { 

#if  __FACE_COLOR_TRACKER_DISPLAY__
    // OpenCV: pop up an OpenCV highgui window
    cvNamedWindow("Face color tracker: Disparity",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Face color tracker: Face Detection", CV_WINDOW_AUTOSIZE);
#endif

    calib_color_ = calib_color;

    people_ = new People();

    // Subscribe to the images and parameters
    sync_.subscribe("stereo/left/image_rect_color",limage_,1);
    sync_.subscribe("stereo/disparity",dimage_,1);
    sync_.subscribe("stereo/stereo_info",stinfo_,1);
    sync_.subscribe("stereo/disparity_info",dispinfo_,1);
    sync_.subscribe("stereo/right/cam_info",rcinfo_,1);
    sync_.ready();

    // Advertise a 3d position measurement for each head.
    advertise<people::PositionMeasurement>("people_tracker_measurements",1);

    if (!detect_faces_) {
      subscribe<people::PositionMeasurement>("people_tracker_filter",init_pos_, &StereoFaceColorTracker::init_pos_cb,1);
    }

  }

  ~StereoFaceColorTracker()
  {

    cvReleaseImage(&cv_image_disp_out_); cv_image_disp_out_ = 0;

#if  __FACE_COLOR_TRACKER_DISPLAY__
    cvDestroyWindow("Face color tracker: Face Detection");
    cvDestroyWindow("Face color tracker: Disparity");
#endif

    delete cam_model_;

    cvReleaseImage(&X_); X_ = 0;
    cvReleaseImage(&Y_); Y_ = 0;
    cvReleaseImage(&Z_); Z_ = 0;
    cvReleaseMat(&uvd_); uvd_ = 0;
    cvReleaseMat(&xyz_); xyz_ = 0;

    delete people_;
  }


  // Callback to (re)initialize the location of a face.
  void init_pos_cb() {

    boost::mutex::scoped_lock lock(cv_mutex_);

    // Check that the timestamp is close to the current time. If not, just wait for another message.
    if ((init_pos_.header.stamp - last_image_time_) < ros::Duration().fromSec(-2.0)) {//2.0)) {
      ROS_INFO_STREAM_NAMED("color_tracker","Initial position is too old.");
      return;
    }

    // Check that we have the camera parameters. Otherwise, we can't convert the position
    // to image coords anyway, so we might as well ignore this message.
    if (cam_model_ == NULL) {
      ROS_INFO_STREAM_NAMED("color_tracker","No camera model.");
      return;
    }

    // Find the person this position message refers to.
    int iperson = people_->findID(init_pos_.object_id);
    CvMat *four_corners_3d = cvCreateMat(4,3,CV_32FC1);
    CvMat *four_corners_2d = cvCreateMat(4,3,CV_32FC1);
    CvMat *center_3d = cvCreateMat(1,3,CV_32FC1);
    CvMat *size_3d = cvCreateMat(1,3,CV_32FC1);
    bool setface = true;
    if (iperson == -1) {
      // This is a new person, create them. Their histogram will be sent on the first tracking frame.
      people_->addPerson();
      iperson = people_->getNumPeople()-1;
      people_->setID(init_pos_.object_id,iperson);
      people_->setFaceSize3D(0, iperson);
      setface = true;
    }
    else {
      // This is a person we've seen before.
      // Clear their face histogram.
      //people_->clearFaceColorHist(iperson);
    }
    if (setface) {

      tf::Point pt;
      tf::pointMsgToTF(init_pos_.pos, pt);
      tf::Stamped<tf::Point> loc(pt, init_pos_.header.stamp, init_pos_.header.frame_id);
      try
      {
	  tf.transformPoint(limage_.header.frame_id, init_pos_.header.stamp, loc, "odom_combined", loc); 
	  // Converts things through the fixed frame into a new time. Use this if you want to go to a different time than loc already has.
      } 
      catch (tf::TransformException& ex)
      {
      }

      people_->setFaceCenter3D(loc[0],loc[1],loc[2],iperson);
      ROS_INFO_STREAM_NAMED("color_tracker","From filter: " );
      people_->printFaceCenter3D(iperson);     
      cvmSet(center_3d,0,0,loc[0]);
      cvmSet(center_3d,0,1,loc[1]);
      cvmSet(center_3d,0,2,loc[2]);
      cvSet(size_3d,cvScalar(people_->getFaceSize3D(iperson)));
      people_->centerSizeToFourCorners(center_3d,size_3d,four_corners_3d);
      cam_model_->cartToDisp(four_corners_3d, four_corners_2d);
      people_->setFaceBbox2D(cvRect(cvmGet(four_corners_2d,0,0),cvmGet(four_corners_2d,0,1),
				    cvmGet(four_corners_2d,1,0)-cvmGet(four_corners_2d,0,0),
				    cvmGet(four_corners_2d,2,1)-cvmGet(four_corners_2d,0,1)),
			     iperson);
    }
    people_->setTrackingFilterUpdateTime(init_pos_.header.stamp, iperson);

    cvReleaseMat(&four_corners_3d);
    cvReleaseMat(&four_corners_2d);
    cvReleaseMat(&center_3d);
    cvReleaseMat(&size_3d);
  }

  /// The image callback when not all topics are sync'ed. Don't do anything, just wait for sync.
  void image_cb_timeout(ros::Time t) {
    ROS_DEBUG_STREAM_NAMED("color_tracker","Message timeout");
  }

  /// The image callback. For each new image, copy it, perform face detection or track it, and draw the rectangles on the image.
  void image_cb_all(ros::Time t)
  {

    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    ros::Time startt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);

    boost::mutex::scoped_lock lock(cv_mutex_);
 
    last_image_time_ = limage_.header.stamp;
    
    // Set color calibration.
    bool do_calib = false;
    if (!calib_color_) {
      if (lbridge_.fromImage(limage_,"bgr8")) {
	cv_image_left_ = lbridge_.toIpl();
	cvSmooth(cv_image_left_, cv_image_left_, CV_GAUSSIAN, 5);
      }
    }
    else if (calib_color_ && lcolor_cal_.getFromParam("stereo/left/image_rect_color")) {
      // Exit if color calibration hasn't been performed.
      do_calib = true;
      if (lbridge_.fromImage(limage_,"bgr8")) {
	cv_image_left_ = lbridge_.toIpl();
	cvSmooth(cv_image_left_, cv_image_left_, CV_GAUSSIAN, 5);
	lcolor_cal_.correctColor(cv_image_left_, cv_image_left_, true, true, COLOR_CAL_BGR);
      }
    }
    else {
      return;
    }
    
    // Set the disparity image.
    if (dbridge_.fromImage(dimage_)) {
      cv_image_disp_ = dbridge_.toIpl();
    }
    

    // Convert the stereo calibration into a camera model.
    if (cam_model_) {
      delete cam_model_;
    }
    double Fx = rcinfo_.P[0];
    double Fy = rcinfo_.P[5];
    double Clx = rcinfo_.P[2];
    double Crx = Clx;
    double Cy = rcinfo_.P[6];
    double Tx = -rcinfo_.P[3]/Fx;
    cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,1.0/dispinfo_.dpp);

    CvSize im_size = cvGetSize(cv_image_left_);

    // Kill anyone who hasn't received a filter update in a long time.
    people_->killIfFilterUpdateTimeout(limage_.header.stamp);

    int npeople = people_->getNumPeople();

    if (npeople==0) {
      // No people to track, try again later.
      return;
    }

    CvMat *end_points = cvCreateMat(npeople,3,CV_32FC1);
    double kernel_size_m = 0.40;//0.7;//0.4; 
    bool tracked_each[npeople];
    bool did_track = people_->track_color_3d_bhattacharya(cv_image_left_, cv_image_disp_, cam_model_, kernel_size_m, 0, NULL, NULL, end_points, tracked_each);//0.3
    if (!did_track) {
      // If tracking failed, just return.
      return;
    }
    // Copy endpoints and the new 2d bbox to the people structure.
    // Draw the points on the image.
    CvMat *four_corners_3d = cvCreateMat(4,3,CV_32FC1);
    CvMat *four_corners_2d = cvCreateMat(4,3,CV_32FC1); 
    CvMat *my_end_point = cvCreateMat(1,3,CV_32FC1);
    CvMat *my_size = cvCreateMat(1,2,CV_32FC1);
    people::PositionMeasurement pos;

    for (int iperson = 0; iperson < npeople; iperson++) {
      if (!tracked_each[iperson]) {
	continue;
      }
      people_->setFaceCenter3D(cvmGet(end_points,iperson,0),cvmGet(end_points,iperson,1),cvmGet(end_points,iperson,2),iperson);
      // Convert the new center and size to a 3d rect that is parallel with the camera plane.
      cvSet(my_size,cvScalar(people_->getFaceSize3D(iperson)));
      ROS_DEBUG_STREAM_NAMED("color_tracker","Face size before disp " << people_->getFaceSize3D(iperson));
      cvGetRow(end_points,my_end_point,iperson);
      people_->centerSizeToFourCorners(my_end_point,my_size, four_corners_3d);
      for (int icorner = 0; icorner < 4; icorner++) {
	cvmSet(four_corners_3d,icorner,2,cvmGet(my_end_point,0,2));
      }
      cam_model_->cartToDisp(four_corners_3d, four_corners_2d);
      people_->setFaceBbox2D(cvRect(cvmGet(four_corners_2d,0,0),cvmGet(four_corners_2d,0,1),
				    cvmGet(four_corners_2d,1,0)-cvmGet(four_corners_2d,0,0),
				    cvmGet(four_corners_2d,2,1)-cvmGet(four_corners_2d,0,1)),
			     iperson);
      cvRectangle(cv_image_left_, 
		  cvPoint(cvmGet(four_corners_2d,0,0),cvmGet(four_corners_2d,0,1)), 
		  cvPoint(cvmGet(four_corners_2d,3,0),cvmGet(four_corners_2d,3,1)),
		  cvScalar(255,255,255)); 

      // Publish the 3d head center for this person.
      pos.header.stamp = limage_.header.stamp;
      pos.name = "stereo_face_color_tracker";
      pos.object_id = people_->getID(iperson);
      pos.pos.x = cvmGet(end_points,iperson,0);
      pos.pos.y = cvmGet(end_points,iperson,1);
      pos.pos.z = cvmGet(end_points,iperson,2);
      pos.header.frame_id = limage_.header.frame_id;//"stereo_optical_frame";
      pos.reliability = 0.5;
      pos.initialization = 0;
      pos.covariance[0] = 0.09; pos.covariance[1] = 0.0;  pos.covariance[2] = 0.0;
      pos.covariance[3] = 0.0;  pos.covariance[4] = 0.09; pos.covariance[5] = 0.0;
      pos.covariance[6] = 0.0;  pos.covariance[7] = 0.0;  pos.covariance[8] = 0.09;
      publish("people_tracker_measurements",pos);
    }
	  
    cvReleaseMat(&end_points);
    cvReleaseMat(&four_corners_3d);
    cvReleaseMat(&four_corners_2d);
    cvReleaseMat(&my_end_point);
    cvReleaseMat(&my_size);     

    gettimeofday(&timeofday,NULL);
    ros::Time endt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
    ros::Duration diff = endt-startt;
    ROS_DEBUG_STREAM_NAMED("color_tracker","Start " << startt.toSec() << " End " << endt.toSec() << " Duration " << diff.toSec() );

#if  __FACE_COLOR_TRACKER_DISPLAY__
    //if (!cv_image_disp_out_) {
    //  cv_image_disp_out_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
    //}
    //cvCvtScale(cv_image_disp_,cv_image_disp_out_,4.0/dispinfo_.dpp);
    cvShowImage("Face color tracker: Face Detection", cv_image_left_);
    //cvShowImage("Face color tracker: Disparity", cv_image_disp_out_);
#endif
    lock.unlock();

  }


  // Wait for completion, wait for user input, display images.
  bool spin() {
    while (ok() && !quit_) {
#if  __FACE_COLOR_TRACKER_DISPLAY__
      boost::mutex::scoped_lock lock(cv_mutex_);
      // Get user input and allow OpenCV to refresh windows.
      int c = cvWaitKey(2);
      c &= 0xFF;
      // Quit on ESC, "q" or "Q"
      if((c == 27)||(c == 'q')||(c == 'Q'))
	quit_ = true;
      lock.unlock();
#endif
      usleep(10000);
    }
    return true;
  } 

};


// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);
  bool use_depth = true;
  bool color_calib = false;
  bool detect_faces = false;

  if (argc > 1) {
    detect_faces = atoi(argv[1]);
  }
  char haar_filename[] = "./cascades/haarcascade_frontalface_alt.xml";
  StereoFaceColorTracker sfct(detect_faces, haar_filename, use_depth, color_calib);
 
  sfct.spin();


  
  return 0;
}


