/**********************************************************************
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



#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include "opencv/cxcore.hpp"
#include "opencv/cv.hpp"
#include "opencv/highgui.h"

#include "face_detector/faces.h"

#include "image_geometry/stereo_camera_model.h"

#include <actionlib/server/simple_action_server.h>
#include <face_detector/FaceDetectorAction.h>

using namespace std;

namespace people
{

/** FaceDetector - A wrapper around OpenCV's face detection, plus some usage of depth from stereo to restrict the results based on plausible face size.
 */
class FaceDetector
{
public:
  // Constants
  const double BIGDIST_M;// = 1000000.0;

  // Node handle
  ros::NodeHandle nh_;

  boost::mutex connect_mutex_;
  bool use_rgbd_;

  // Subscription topics
  string image_image_;
  // Stereo
  string stereo_namespace_;
  string left_topic_;
  string disparity_topic_;
  string left_camera_info_topic_;
  string right_camera_info_topic_;
  // RGB-D
  string camera_;
  string camera_topic_;
  string depth_image_;
  string depth_topic_;
  string camera_info_topic_;
  string depth_info_topic_;
  string depth_ns_;

  // Images and conversion for both the stereo camera and rgb-d camera cases.
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter image_sub_; /**< Left/rgb image msg. */
  image_transport::SubscriberFilter depth_image_sub_; /** Depth image msg. */
  message_filters::Subscriber<stereo_msgs::DisparityImage> disp_image_sub_; /**< Disparity image msg. */
  message_filters::Subscriber<sensor_msgs::CameraInfo> c1_info_sub_; /**< Left/rgb camera info msg. */
  message_filters::Subscriber<sensor_msgs::CameraInfo> c2_info_sub_; /**< Right/depth camera info msg. */

  // Disparity:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactDispPolicy; /**< Sync policy for exact time with disparity. */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateDispPolicy; /**< Sync policy for approx time with disparity. */
  typedef message_filters::Synchronizer<ExactDispPolicy> ExactDispSync;
  typedef message_filters::Synchronizer<ApproximateDispPolicy> ApproximateDispSync;
  boost::shared_ptr<ExactDispSync> exact_disp_sync_;
  boost::shared_ptr<ApproximateDispSync> approximate_disp_sync_;

  // Depth:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactDepthPolicy; /**< Sync policy for exact time with depth. */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateDepthPolicy; /**< Sync policy for approx time with depth. */
  typedef message_filters::Synchronizer<ExactDepthPolicy> ExactDepthSync;
  typedef message_filters::Synchronizer<ApproximateDepthPolicy> ApproximateDepthSync;
  boost::shared_ptr<ExactDepthSync> exact_depth_sync_;
  boost::shared_ptr<ApproximateDepthSync> approximate_depth_sync_;

  // Action
  actionlib::SimpleActionServer<face_detector::FaceDetectorAction> as_;
  face_detector::FaceDetectorFeedback feedback_;
  face_detector::FaceDetectorResult result_;

  // If running the face detector as a component in part of a larger person tracker, this subscribes to the tracker's position measurements and whether it was initialized by some other node.
  // Todo: resurrect the person tracker.
  ros::Subscriber pos_sub_;
  bool external_init_;


  // Publishers
  // A point cloud of the face positions, meant for visualization in rviz.
  // This could be replaced by visualization markers, but they can't be modified
  // in rviz at runtime (eg the alpha, display time, etc. can't be changed.)
  ros::Publisher cloud_pub_;
  ros::Publisher pos_array_pub_;

  // Display
  bool do_display_; /**< True/false display images with bounding boxes locally. */
  cv::Mat cv_image_out_; /**< Display image. */

  // Depth
  bool use_depth_; /**< True/false use depth information. */
  image_geometry::StereoCameraModel cam_model_; /**< ROS->OpenCV image_geometry conversion. */

  // Face detector params and output
  Faces *faces_; /**< List of faces and associated fcns. */
  string name_; /**< Name of the detector. Ie frontalface, profileface. These will be the names in the published face location msgs. */
  string haar_filename_; /**< Training file for the haar cascade classifier. */
  double reliability_; /**< Reliability of the predictions. This should depend on the training file used. */

  struct RestampedPositionMeasurement
  {
    ros::Time restamp;
    people_msgs::PositionMeasurement pos;
    double dist;
  };
  map<string, RestampedPositionMeasurement> pos_list_; /**< Queue of updated face positions from the filter. */

  int max_id_;

  bool quit_;

  tf::TransformListener tf_;

  std::string fixed_frame_;

  boost::mutex cv_mutex_, pos_mutex_, limage_mutex_, dimage_mutex_;

  bool do_continuous_; /**< True = run as a normal node, searching for faces continuously, False = run as an action, wait for action call to start detection. */

  bool do_publish_unknown_; /**< Publish faces even if they have unknown depth/size. Will just use the image x,y in the pos field of the published position_measurement. */

  FaceDetector(std::string name) :
    BIGDIST_M(1000000.0),
    it_(nh_),
    as_(nh_, name, false),
    faces_(0),
    max_id_(-1),
    quit_(false)
  {
    ROS_INFO_STREAM_NAMED("face_detector", "Constructing FaceDetector.");

    // Action stuff
    as_.registerGoalCallback(boost::bind(&FaceDetector::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FaceDetector::preemptCB, this));
    as_.start();

    faces_ = new Faces();
    double face_size_min_m, face_size_max_m, max_face_z_m, face_sep_dist_m;
    int queue_size;
    bool approx;

    // Parameters
    ros::NodeHandle local_nh("~");
    local_nh.param("classifier_name", name_, std::string(""));
    local_nh.param("classifier_filename", haar_filename_, std::string(""));
    local_nh.param("classifier_reliability", reliability_, 0.0);
    local_nh.param("do_display", do_display_, false);
    local_nh.param("do_continuous", do_continuous_, true);
    local_nh.param("do_publish_faces_of_unknown_size", do_publish_unknown_, false);
    local_nh.param("use_depth", use_depth_, true);
    local_nh.param("use_external_init", external_init_, false);
    local_nh.param("face_size_min_m", face_size_min_m, FACE_SIZE_MIN_M);
    local_nh.param("face_size_max_m", face_size_max_m, FACE_SIZE_MAX_M);
    local_nh.param("max_face_z_m", max_face_z_m, MAX_FACE_Z_M);
    local_nh.param("face_separation_dist_m", face_sep_dist_m, FACE_SEP_DIST_M);
    local_nh.param("use_rgbd", use_rgbd_, false);
    local_nh.param("queue_size", queue_size, 5);
    local_nh.param("approximate_sync", approx, false);

    if (do_display_)
    {
      // OpenCV: pop up an OpenCV highgui window
      cv::namedWindow("Face detector: Face Detection", CV_WINDOW_AUTOSIZE);
    }

    // Init the detector and subscribe to the images and camera parameters. One case for rgbd, one for stereo.
    if (use_rgbd_)
    {

      faces_->initFaceDetectionDepth(1, haar_filename_, face_size_min_m, face_size_max_m, max_face_z_m, face_sep_dist_m);

      camera_ = nh_.resolveName("camera");
      image_image_ = nh_.resolveName("image_topic");
      depth_image_ = nh_.resolveName("depth_topic");
      depth_ns_ = nh_.resolveName("depth_ns");
      camera_topic_ = ros::names::clean(camera_ + "/rgb/" + image_image_);
      depth_topic_ = ros::names::clean(camera_ + "/" + depth_ns_ + "/" + depth_image_);
      camera_info_topic_ = ros::names::clean(camera_ + "/rgb/camera_info");
      depth_info_topic_ = ros::names::clean(camera_ + "/" + depth_ns_ + "/camera_info");

      local_nh.param("fixed_frame", fixed_frame_, std::string("camera_rgb_optical_frame"));

      if (approx)
      {
        approximate_depth_sync_.reset(new ApproximateDepthSync(ApproximateDepthPolicy(queue_size),
                                      image_sub_, depth_image_sub_, c1_info_sub_, c2_info_sub_));
        approximate_depth_sync_->registerCallback(boost::bind(&FaceDetector::imageCBAllDepth,
            this, _1, _2, _3, _4));
      }
      else
      {
        exact_depth_sync_.reset(new ExactDepthSync(ExactDepthPolicy(queue_size),
                                image_sub_, depth_image_sub_, c1_info_sub_, c2_info_sub_));
        exact_depth_sync_->registerCallback(boost::bind(&FaceDetector::imageCBAllDepth,
                                            this, _1, _2, _3, _4));
      }

    }
    else
    {
      faces_->initFaceDetectionDisparity(1, haar_filename_, face_size_min_m, face_size_max_m, max_face_z_m, face_sep_dist_m);

      stereo_namespace_ = nh_.resolveName("camera");
      image_image_ = nh_.resolveName("image");
      left_topic_ = ros::names::clean(stereo_namespace_ + "/left/" + image_image_);
      disparity_topic_ = ros::names::clean(stereo_namespace_ + "/disparity");
      left_camera_info_topic_ = ros::names::clean(stereo_namespace_ + "/left/camera_info");
      right_camera_info_topic_ = ros::names::clean(stereo_namespace_ + "/right/camera_info");

      local_nh.param("fixed_frame", fixed_frame_, stereo_namespace_.append("_optical_frame"));

      if (approx)
      {
        approximate_disp_sync_.reset(new ApproximateDispSync(ApproximateDispPolicy(queue_size),
                                     image_sub_, disp_image_sub_, c1_info_sub_, c2_info_sub_));
        approximate_disp_sync_->registerCallback(boost::bind(&FaceDetector::imageCBAllDisp,
            this, _1, _2, _3, _4));
      }
      else
      {
        exact_disp_sync_.reset(new ExactDispSync(ExactDispPolicy(queue_size),
                               image_sub_, disp_image_sub_, c1_info_sub_, c2_info_sub_));
        exact_disp_sync_->registerCallback(boost::bind(&FaceDetector::imageCBAllDisp,
                                           this, _1, _2, _3, _4));
      }
    }

    // Connection callbacks and advertise
    ros::SubscriberStatusCallback pos_pub_connect_cb = boost::bind(&FaceDetector::connectCb, this);
    ros::SubscriberStatusCallback cloud_pub_connect_cb = boost::bind(&FaceDetector::connectCb, this);
    if (do_continuous_)
      ROS_INFO_STREAM_NAMED("face_detector", "You must subscribe to one of FaceDetector's outbound topics or else it will not publish anything.");

    {
      boost::mutex::scoped_lock lock(connect_mutex_);

      // Advertise a position measure message.
      pos_array_pub_ = nh_.advertise<people_msgs::PositionMeasurementArray>("face_detector/people_tracker_measurements_array", 1, pos_pub_connect_cb, pos_pub_connect_cb);

      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("face_detector/faces_cloud", 0, cloud_pub_connect_cb, cloud_pub_connect_cb);
    }


    // If running as an action server, just stay connected so that action calls are fast.
    if (!do_continuous_) connectCb();

    ros::MultiThreadedSpinner s(2);
    ros::spin(s);

  }

  ~FaceDetector()
  {

    cv_image_out_.release();

    if (do_display_)
    {
      cv::destroyWindow("Face detector: Face Detection");
    }

    if (faces_)
    {
      delete faces_;
      faces_ = 0;
    }
  }

  // Handles (un)subscribing when clients (un)subscribe
  void connectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (use_rgbd_)
    {
      if (do_continuous_ && cloud_pub_.getNumSubscribers() == 0 && pos_array_pub_.getNumSubscribers() == 0)
      {
        ROS_INFO_STREAM_NAMED("face_detector", "You have unsubscribed to FaceDetector's outbound topics, so it will no longer publish anything.");
        image_sub_.unsubscribe();
        depth_image_sub_.unsubscribe();
        c1_info_sub_.unsubscribe();
        c2_info_sub_.unsubscribe();
        pos_sub_.shutdown();
      }
      else if (!do_continuous_ || !image_sub_.getSubscriber())
      {
        image_sub_.subscribe(it_, camera_topic_, 3);
        depth_image_sub_.subscribe(it_, depth_topic_, 3);
        c1_info_sub_.subscribe(nh_, camera_info_topic_, 3);
        c2_info_sub_.subscribe(nh_, depth_info_topic_, 3);
        // // Subscribe to filter measurements.
        // if (external_init_) {
        //   //pos_sub_ = nh_.subscribe("people_tracker_filter",1,&FaceDetector::posCallback,this);
        //   pos_sub_ = nh_.subscribe("/face_detector/people_tracker_measurements_array",1,&FaceDetector::posCallback,this);
        // }
      }
    }
    else
    {
      if (do_continuous_ && cloud_pub_.getNumSubscribers() == 0 && pos_array_pub_.getNumSubscribers() == 0)
      {
        ROS_INFO_STREAM_NAMED("face_detector", "You have unsubscribed to FaceDetector's outbound topics, so it will no longer publish anything.");
        image_sub_.unsubscribe();
        disp_image_sub_.unsubscribe();
        c1_info_sub_.unsubscribe();
        c2_info_sub_.unsubscribe();
        pos_sub_.shutdown();
      }
      else if (!do_continuous_ || !image_sub_.getSubscriber())
      {
        image_sub_.subscribe(it_, left_topic_, 3);
        disp_image_sub_.subscribe(nh_, disparity_topic_, 3);
        c1_info_sub_.subscribe(nh_, left_camera_info_topic_, 3);
        c2_info_sub_.subscribe(nh_, right_camera_info_topic_, 3);
        // // Subscribe to filter measurements.
        // if (external_init_) {
        //   //pos_sub_ = nh_.subscribe("people_tracker_filter",1,&FaceDetector::posCallback,this);
        //   pos_sub_ = nh_.subscribe("/face_detector/people_tracker_measurements_array",1,&FaceDetector::posCallback,this);
        // }
      }
    }
  }


  void goalCB()
  {
    as_.acceptNewGoal();
  }

  void preemptCB()
  {
    as_.setPreempted();
  }


  /*!
   * \brief Position message callback.
   *
   * When hooked into the person tracking filter, this callback will listen to messages
   * from the filter with an array of person ids and 3D positions and adjust those people's face positions accordingly.
   */
  // void posCallback(const people_msgs::PositionMeasurementArrayConstPtr& pos_array_ptr) {

  //   // Put the incoming position into the position queue. It'll be processed in the next image callback.
  //   boost::mutex::scoped_lock lock(pos_mutex_);

  //   map<string, RestampedPositionMeasurement>::iterator it;
  //   for (uint ipa = 0; ipa < pos_array_ptr->people.size(); ipa++) {
  //     it = pos_list_.find(pos_array_ptr->people[ipa].object_id);
  //     RestampedPositionMeasurement rpm;
  //     rpm.pos = pos_array_ptr->people[ipa];
  //     rpm.restamp = pos_array_ptr->people[ipa].header.stamp;
  //     rpm.dist = BIGDIST_M;
  //     if (it == pos_list_.end()) {
  //  pos_list_.insert(pair<string, RestampedPositionMeasurement>(pos_array_ptr->people[ipa].object_id, rpm));
  //     }
  //     else if ((pos_array_ptr->people[ipa].header.stamp - (*it).second.pos.header.stamp) > ros::Duration().fromSec(-1.0) ){
  //  (*it).second = rpm;
  //     }
  //   }
  //   lock.unlock();

  // }

  // Workaround to convert a DisparityImage->Image into a shared pointer for cv_bridge in imageCBAll.
  struct NullDeleter
  {
    void operator()(void const *) const {}
  };

  /*!
   * \brief Image callback for synced messages.
   *
   * For each new image:
   * convert it to OpenCV format, perform face detection using OpenCV's haar filter cascade classifier, and
   * (if requested) draw rectangles around the found faces.
   * Can also compute which faces are associated (by proximity, currently) with faces it already has in its list of people.
   */
  void imageCBAllDepth(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info)
  {

    // Only run the detector if in continuous mode or the detector was turned on through an action invocation.
    if (!do_continuous_ && !as_.isActive())
      return;

    // Clear out the result vector.
    result_.face_positions.clear();

    if (do_display_)
    {
      cv_mutex_.lock();
    }

    // ROS --> OpenCV
    cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");
    cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image);
    cv::Mat depth_32fc1 = cv_depth_ptr->image;
    if (depth_image->encoding != "32FC1")
    {
      cv_depth_ptr->image.convertTo(depth_32fc1, CV_32FC1, 0.001);
    }



    cam_model_.fromCameraInfo(c1_info, c2_info);

    // For display, keep a copy of the image that we can draw on.
    if (do_display_)
    {
      cv_image_out_ = (cv_image_ptr->image).clone();
    }

    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    ros::Time starttdetect = ros::Time().fromNSec(1e9 * timeofday.tv_sec + 1e3 * timeofday.tv_usec);

    vector<Box2D3D> faces_vector = faces_->detectAllFacesDepth(cv_image_ptr->image, 1.0, depth_32fc1, &cam_model_);
    gettimeofday(&timeofday, NULL);
    ros::Time endtdetect = ros::Time().fromNSec(1e9 * timeofday.tv_sec + 1e3 * timeofday.tv_usec);
    ros::Duration diffdetect = endtdetect - starttdetect;
    ROS_INFO_STREAM_NAMED("face_detector", "Detection duration = " << diffdetect.toSec() << "sec");

    matchAndDisplay(faces_vector, image->header);
  }

  /*!
   * \brief Image callback for synced messages.
   *
   * For each new image:
   * convert it to OpenCV format, perform face detection using OpenCV's haar filter cascade classifier, and
   * (if requested) draw rectangles around the found faces.
   * Can also compute which faces are associated (by proximity, currently) with faces it already has in its list of people.
   */
  void imageCBAllDisp(const sensor_msgs::Image::ConstPtr &image, const stereo_msgs::DisparityImage::ConstPtr& disp_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info)
  {

    // Only run the detector if in continuous mode or the detector was turned on through an action invocation.
    if (!do_continuous_ && !as_.isActive())
      return;

    // Clear out the result vector.
    result_.face_positions.clear();

    if (do_display_)
    {
      cv_mutex_.lock();
    }


    // ROS --> OpenCV
    cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr cv_disp_ptr = cv_bridge::toCvShare(disp_image->image, disp_image);
    cam_model_.fromCameraInfo(c1_info, c2_info);

    // For display, keep a copy of the image that we can draw on.
    if (do_display_)
    {
      cv_image_out_ = (cv_image_ptr->image).clone();
    }

    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    ros::Time starttdetect = ros::Time().fromNSec(1e9 * timeofday.tv_sec + 1e3 * timeofday.tv_usec);

    vector<Box2D3D> faces_vector = faces_->detectAllFacesDisparity(cv_image_ptr->image, 1.0, cv_disp_ptr->image, &cam_model_);
    gettimeofday(&timeofday, NULL);
    ros::Time endtdetect = ros::Time().fromNSec(1e9 * timeofday.tv_sec + 1e3 * timeofday.tv_usec);
    ros::Duration diffdetect = endtdetect - starttdetect;
    ROS_INFO_STREAM_NAMED("face_detector", "Detection duration = " << diffdetect.toSec() << "sec");

    matchAndDisplay(faces_vector, image->header);
  }


private:

  void matchAndDisplay(vector<Box2D3D> faces_vector, std_msgs::Header header)
  {
    bool found_faces = false;

    int ngood = 0;
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = header.stamp;
    cloud.header.frame_id = header.frame_id;

    if (faces_vector.size() > 0)
    {

      // Transform the positions of the known faces and remove anyone who hasn't had an update in a long time.
      //      boost::mutex::scoped_lock pos_lock(pos_mutex_);
      map<string, RestampedPositionMeasurement>::iterator it;
      it = pos_list_.begin();
      while (it != pos_list_.end())
      {
        if ((header.stamp - (*it).second.restamp) > ros::Duration().fromSec(5.0))
        {
          // Position is too old, remove the person.
          pos_list_.erase(it++);
        }
        else
        {
          // Transform the person to this time. Note that the pos time is updated but not the restamp.
          tf::Point pt;
          tf::pointMsgToTF((*it).second.pos.pos, pt);
          tf::Stamped<tf::Point> loc(pt, (*it).second.pos.header.stamp, (*it).second.pos.header.frame_id);
          try
          {
            tf_.transformPoint(header.frame_id, header.stamp, loc, fixed_frame_, loc);
            (*it).second.pos.header.stamp = header.stamp;
            (*it).second.pos.pos.x = loc[0];
            (*it).second.pos.pos.y = loc[1];
            (*it).second.pos.pos.z = loc[2];
          }
          catch (tf::TransformException& ex)
          {
          }
          it++;
        }
      }
      // End filter face position update

      // Associate the found faces with previously seen faces, and publish all good face centers.
      Box2D3D *one_face;
      people_msgs::PositionMeasurement pos;
      people_msgs::PositionMeasurementArray pos_array;

      for (uint iface = 0; iface < faces_vector.size(); iface++)
      {
        one_face = &faces_vector[iface];

        if (one_face->status == "good" || (one_face->status == "unknown" && do_publish_unknown_))
        {

          std::string id = "";

          // Convert the face format to a PositionMeasurement msg.
          pos.header.stamp = header.stamp;
          pos.name = name_;
          pos.pos.x = one_face->center3d.x;
          pos.pos.y = one_face->center3d.y;
          pos.pos.z = one_face->center3d.z;
          pos.header.frame_id = header.frame_id;//"*_optical_frame";
          pos.reliability = reliability_;
          pos.initialization = 1;//0;
          pos.covariance[0] = 0.04;
          pos.covariance[1] = 0.0;
          pos.covariance[2] = 0.0;
          pos.covariance[3] = 0.0;
          pos.covariance[4] = 0.04;
          pos.covariance[5] = 0.0;
          pos.covariance[6] = 0.0;
          pos.covariance[7] = 0.0;
          pos.covariance[8] = 0.04;

          // Check if this person's face is close enough to one of the previously known faces and associate it with the closest one.
          // Otherwise publish it with an empty id.
          // Note that multiple face positions can be published with the same id, but ids in the pos_list_ are unique. The position of a face in the list is updated with the closest found face.
          double dist, mindist = BIGDIST_M;
          map<string, RestampedPositionMeasurement>::iterator close_it = pos_list_.end();
          for (it = pos_list_.begin(); it != pos_list_.end(); it++)
          {
            dist = pow((*it).second.pos.pos.x - pos.pos.x, 2.0) + pow((*it).second.pos.pos.y - pos.pos.y, 2.0) + pow((*it).second.pos.pos.z - pos.pos.z, 2.0);
            if (dist <= faces_->face_sep_dist_m_ && dist < mindist)
            {
              mindist = dist;
              close_it = it;
            }
          }
          if (close_it != pos_list_.end())
          {
            pos.object_id = (*close_it).second.pos.object_id;
            if (mindist < (*close_it).second.dist)
            {
              (*close_it).second.restamp = header.stamp;
              (*close_it).second.dist = mindist;
              (*close_it).second.pos = pos;
            }
            ROS_INFO_STREAM_NAMED("face_detector", "Found face to match with id " << pos.object_id);
          }
          else
          {
            max_id_++;
            pos.object_id = static_cast<ostringstream*>(&(ostringstream() << max_id_))->str();
            ROS_INFO_STREAM_NAMED("face_detector", "Didn't find face to match, starting new ID " << pos.object_id);
          }
          result_.face_positions.push_back(pos);
          found_faces = true;

        }

      }
      pos_array.header.stamp = header.stamp;
      pos_array.header.frame_id = header.frame_id;
      pos_array.people = result_.face_positions;
      if (pos_array.people.size() > 0)
      {
        pos_array_pub_.publish(pos_array);

        // Update the position list greedily. This should be rewritten.
        map<string, RestampedPositionMeasurement>::iterator it;
        for (uint ipa = 0; ipa < pos_array.people.size(); ipa++)
        {
          it = pos_list_.find(pos_array.people[ipa].object_id);
          RestampedPositionMeasurement rpm;
          rpm.pos = pos_array.people[ipa];
          rpm.restamp = pos_array.people[ipa].header.stamp;
          rpm.dist = BIGDIST_M;
          if (it == pos_list_.end())
          {
            pos_list_.insert(pair<string, RestampedPositionMeasurement>(pos_array.people[ipa].object_id, rpm));
          }
          else if ((pos_array.people[ipa].header.stamp - (*it).second.pos.header.stamp) > ros::Duration().fromSec(-1.0))
          {
            (*it).second = rpm;
          }
        }
        //      pos_lock.unlock();

        // Clean out all of the distances in the pos_list_
        for (it = pos_list_.begin(); it != pos_list_.end(); it++)
        {
          (*it).second.dist = BIGDIST_M;
        }

      }


      // Done associating faces

      /******** Display **************************************************************/

      // Publish a point cloud of face centers.

      cloud.channels.resize(1);
      cloud.channels[0].name = "intensity";

      for (uint iface = 0; iface < faces_vector.size(); iface++)
      {
        one_face = &faces_vector[iface];

        // Visualization of good faces as a point cloud
        if (one_face->status == "good")
        {

          geometry_msgs::Point32 p;
          p.x = one_face->center3d.x;
          p.y = one_face->center3d.y;
          p.z = one_face->center3d.z;
          cloud.points.push_back(p);
          cloud.channels[0].values.push_back(1.0f);

          ngood ++;
        }
      }

      cloud_pub_.publish(cloud);
      // Done publishing the point cloud.

    } // Done if faces_vector.size() > 0


    // Draw an appropriately colored rectangle on the display image and in the visualizer.
    if (do_display_)
    {
      displayFacesOnImage(faces_vector);
      cv_mutex_.unlock();
    }
    // Done drawing.

    /******** Done display **********************************************************/

    ROS_INFO_STREAM_NAMED("face_detector", "Number of faces found: " << faces_vector.size() << ", number with good depth and size: " << ngood);

    // If you don't want continuous processing and you've found at least one face, turn off the detector.
    if (!do_continuous_ && found_faces)
    {
      as_.setSucceeded(result_);
    }
  }

  // Draw bounding boxes around detected faces on the cv_image_out_ and show the image.
  void displayFacesOnImage(vector<Box2D3D>  faces_vector)
  {

    Box2D3D *one_face;

    for (uint iface = 0; iface < faces_vector.size(); iface++)
    {
      one_face = &faces_vector[iface];
      // Visualization by image display.
      if (do_display_)
      {
        cv::Scalar color;
        if (one_face->status == "good")
        {
          color = cv::Scalar(0, 255, 0);
        }
        else if (one_face->status == "unknown")
        {
          color = cv::Scalar(255, 0, 0);
        }
        else
        {
          color = cv::Scalar(0, 0, 255);
        }

        cv::rectangle(cv_image_out_,
                      cv::Point(one_face->box2d.x, one_face->box2d.y),
                      cv::Point(one_face->box2d.x + one_face->box2d.width, one_face->box2d.y + one_face->box2d.height), color, 4);
      }
    }

    cv::imshow("Face detector: Face Detection", cv_image_out_);
    cv::waitKey(2);

  }


}; // end class

}; // end namespace people

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_detector");

  people::FaceDetector fd(ros::this_node::getName());

  return 0;
}
