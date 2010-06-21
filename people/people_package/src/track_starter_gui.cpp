/*********************************************************************
* A GUI for clicking on points which are then published as track initialization points.
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
#include <vector>

#include "ros/node.h"
#include "stereo_msgs/StereoInfo.h"
#include "stereo_msgs/DisparityInfo.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include "CvStereoCamModel.h"
#include <people_package/PositionMeasurement.h>
#include "color_calib.h"
#include "topic_synchronizer/topic_synchronizer.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <boost/thread/mutex.hpp>

using namespace std;

struct PublishedPoint {
  CvPoint xy;
  bool published;
};

vector<PublishedPoint> gxys;
boost::mutex g_selection_mutex;
ros::Time g_last_image_time;
bool g_do_cb;

using namespace std;


void on_mouse(int event, int x, int y, int flags, void *params){
  
  if (event==CV_EVENT_MOUSEMOVE) {
  }
  else if (event==CV_EVENT_LBUTTONDOWN) {
    // Add a clicked-on point to the list of points, to be published by the image callback on the next image.
    boost::mutex::scoped_lock sel_lock(g_selection_mutex);
    PublishedPoint p;
    p.xy = cvPoint(x,y);
    p.published = false;
    gxys.push_back(p);
    g_do_cb = true;
    sel_lock.unlock();
  }
  else {
  }
  
}

/**
 * This node provides a GUI for starting tracker tracks. Press P to pause the playback and click on a point in the image. 
 * The 3D coords of the point will be published. 
 */
class TrackStarterGUI: public ros::Node
{
public:
  sensor_msgs::Image limage_;
  sensor_msgs::Image dimage_;
  stereo_msgs::StereoInfo stinfo_;
  stereo_msgs::DisparityInfo dispinfo_;
  sensor_msgs::CameraInfo rcinfo_;
  sensor_msgs::CvBridge lbridge_;
  sensor_msgs::CvBridge dbridge_;
  color_calib::Calibration lcolor_cal_;
  bool quit_;
  bool calib_color_;
  CvStereoCamModel *cam_model_;
  CvMat *uvd_, *xyz_;
  IplImage *cv_image_;
  IplImage *cv_disp_image_;
  IplImage *cv_disp_image_out_;
  boost::mutex cv_mutex_;
  people::PositionMeasurement pos;
  TopicSynchronizer<TrackStarterGUI> sync_;
  
  ros::NodeHandle nh_;

  TrackStarterGUI():
    ros::Node("track_starter_gui"),
    lcolor_cal_(nh_),
    quit_(false),
    calib_color_(false),
    cam_model_(NULL),
    uvd_(NULL),
    xyz_(NULL),
    cv_image_(NULL),
    cv_disp_image_(NULL),
    cv_disp_image_out_(NULL),
    sync_(this, &TrackStarterGUI::image_cb_all, ros::Duration().fromSec(0.05), &TrackStarterGUI::image_cb_timeout)
  {
    

    cvNamedWindow("Track Starter: Left Image",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Track Starter: Disparity",CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Track Starter: Left Image", on_mouse, 0);

    uvd_ = cvCreateMat(1,3,CV_32FC1);
    xyz_ = cvCreateMat(1,3,CV_32FC1);

    advertise<people::PositionMeasurement>("people_tracker_measurements",1);
    std::list<std::string> left_list;
    //left_list.push_back(std::string("stereodcam/left/image_rect_color"));
    left_list.push_back(std::string("stereo/left/image_rect"));    
    sync_.subscribe(left_list,limage_,1);
    sync_.subscribe("stereo/disparity",dimage_,1);
    sync_.subscribe("stereo/stereo_info", stinfo_,1);
    sync_.subscribe("stereo/disparity_info", dispinfo_,1);
    sync_.subscribe("stereo/right/cam_info",rcinfo_,1);
    sync_.ready();
    //subscribe("person_measurement",pos,&TrackStarterGUI::point_cb,1);
    
  }

  ~TrackStarterGUI() 
  {

    cvDestroyWindow("Track Starter: Left Image");
    cvDestroyWindow("Track Starter: Disparity");
    
    cvReleaseImage(&cv_disp_image_out_); cv_disp_image_out_ = 0;
    cvReleaseMat(&uvd_); uvd_ = 0;
    cvReleaseMat(&xyz_); xyz_ = 0;

    if (cam_model_) {
      delete cam_model_;
      cam_model_ = 0;
    }
  }

  // Sanity check, print the published point.
  void point_cb() {
    printf("pos %f %f %f\n",pos.pos.x,pos.pos.y,pos.pos.z);

  }

  // Image callback. Draws selected points on images, publishes the point messages, and copies the images to be displayed.
  void image_cb_all(ros::Time t){

    boost::mutex::scoped_lock sel_lock(g_selection_mutex);//cv_mutex_);
    if (!g_do_cb) {
      sel_lock.unlock();
      return;
    }

    sel_lock.unlock();

    bool do_calib = false;
    if (limage_.encoding != "mono8") {
      // If this is a color image, set the calibration and convert it.
      if (calib_color_ && lcolor_cal_.getFromParam("stereo/left/image_rect_color")) {
	do_calib = true;      
      }
      // Convert the images to OpenCV format.
      if (lbridge_.fromImage(limage_,"bgr8")) {
	cv_image_ = lbridge_.toIpl();
	if (do_calib) {
	  lcolor_cal_.correctColor(cv_image_, cv_image_, true, true, COLOR_CAL_BGR);
	}
      }
    }
    else {
      // If this is a mono image, just convert it.
      lbridge_.fromImage(limage_,"mono8");
      cv_image_ = lbridge_.toIpl();
    }

    CvSize im_size = cvGetSize(cv_image_);

    if (dbridge_.fromImage(dimage_)) {
      cv_disp_image_ = dbridge_.toIpl();
      //cvSet(cv_disp_image_,cvScalar(400));
      if (!cv_disp_image_out_) {
	cv_disp_image_out_ = cvCreateImage(im_size,IPL_DEPTH_8U, 1);
      }
      cvCvtScale(cv_disp_image_, cv_disp_image_out_, 4.0/dispinfo_.dpp);
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
    //printf("%f %f %f %f %f %f %f\n", Fx,Fy,Tx,Clx,Crx,Cy,1.0/dispinfo_.dpp);
    cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,1.0/dispinfo_.dpp);

    sel_lock.lock();
    for (uint i = 0; i<gxys.size(); i++) {

      if (cam_model_ && !gxys[i].published) {
	bool search = true;
  
	int d = cvGetReal2D(cv_disp_image_,gxys[i].xy.y,gxys[i].xy.x);
	if (d==0.0) {
	  search = true;
	  int x1 = gxys[i].xy.x;	
	  int y1 = gxys[i].xy.y;
	  int x2 = gxys[i].xy.x;	
	  int y2 = gxys[i].xy.y;
	  while (d==0.0 && search) {
	    x1--; y1--;
	    x2++; y2++;
	    search = false;
	    if (x1 > 0) {
	      d = cvGetReal2D(cv_disp_image_,gxys[i].xy.y,x1);
	      search = true;
	    }
	    if (d==0.0 && x2 < cv_disp_image_->width) {     
	      d = cvGetReal2D(cv_disp_image_,gxys[i].xy.y,x2);
	      search = true;
	    }
	    if (d==0.0 && y1 > 0) {
	      d = cvGetReal2D(cv_disp_image_,y1,gxys[i].xy.x);
	      search = true;
	    }
	    if (d==0.0 && y2 < cv_disp_image_->height) {
	      d = cvGetReal2D(cv_disp_image_,y2,gxys[i].xy.x);
	      search = true;
	    }
	      
	  }
	}
	if (search) {  
	  people::PositionMeasurement pm;
	  pm.header.stamp = g_last_image_time;
	  pm.name = "track_starter_gui";
	  pm.object_id = "";//gxys.size();   
	  cvmSet(uvd_,0,0,gxys[i].xy.x);
	  cvmSet(uvd_,0,1,gxys[i].xy.y);
	  cvmSet(uvd_,0,2,d);
	  cam_model_->dispToCart(uvd_, xyz_);
	  //pm.pos.x = cvmGet(xyz_,0,2);
	  //pm.pos.y = -1.0*cvmGet(xyz_,0,0);
	  //pm.pos.z = -1.0*cvmGet(xyz_,0,1);
	  pm.pos.x = cvmGet(xyz_,0,0);
	  pm.pos.y = cvmGet(xyz_,0,1);
	  pm.pos.z = cvmGet(xyz_,0,2);
	  printf("Publishing %f %f %f\n", cvmGet(xyz_,0,0),cvmGet(xyz_,0,1),cvmGet(xyz_,0,2));
	  pm.header.frame_id = limage_.header.frame_id;//"stereo_link";
	  pm.reliability = 1;
	  pm.initialization = 1;
	  pm.covariance[0] = 0.09;
	  pm.covariance[1] = 0.0;
	  pm.covariance[2] = 0.0;
	  pm.covariance[3] = 0.0;
	  pm.covariance[4] = 0.09;
	  pm.covariance[5] = 0.0;
	  pm.covariance[6] = 0.0;
	  pm.covariance[7] = 0.0;
	  pm.covariance[8] = 0.09;
	  publish("people_tracker_measurements",pm);
	  gxys[i].published = true;
	}	
      }

      cvCircle(cv_image_, gxys[i].xy, 2 , cvScalar(255,0,0) ,4);
      cvCircle(cv_disp_image_out_, gxys[i].xy, 2 , cvScalar(4*255) ,4);
    }

    g_last_image_time = limage_.header.stamp;
    sel_lock.unlock();

    boost::mutex::scoped_lock lock(cv_mutex_);
    cvShowImage("Track Starter: Left Image", cv_image_);
    cvShowImage("Track Starter: Disparity", cv_disp_image_out_);
    lock.unlock();
  }

  // Timeout callback
  void image_cb_timeout(ros::Time t) {   
  }

  // Wait for thread to exit.
  bool spin() {

    while (ok() && !quit_) {
      // Display the image
      boost::mutex::scoped_lock lock(cv_mutex_);
      int c = cvWaitKey(2);
      c &= 0xFF;
      // Quit on ESC, "q" or "Q"
      if((c == 27)||(c == 'q')||(c == 'Q')){
	quit_ = true;
      }
      // Pause playback for point selection on "p" or "P"
      else if ((c=='p') || (c=='P')){
	g_do_cb = 1-g_do_cb;
      }
      lock.unlock();
      usleep(10000);
	
    }
    return true;
  }
};

// Main 
int main(int argc, char**argv) 
{
  ros::init(argc,argv);
 
  g_do_cb = true;
  TrackStarterGUI tsgui;
  tsgui.spin();
  
  return 0;

}
