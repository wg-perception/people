/*********************************************************************
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

/* Author: Wim Meeussen */

#include "people_tracking_filter/people_tracking_node.h"
#include "people_tracking_filter/tracker_particle.h"
#include "people_tracking_filter/tracker_kalman.h"
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"
#include <people_msgs/PositionMeasurement.h>


using namespace std;
using namespace tf;
using namespace BFL;
using namespace message_filters;

static const double       sequencer_delay            = 0.8; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

namespace estimation
{
// constructor
PeopleTrackingNode::PeopleTrackingNode(ros::NodeHandle nh)
  : nh_(nh),
    robot_state_(),
    tracker_counter_(0)
{
  // initialize
  meas_cloud_.points = vector<geometry_msgs::Point32>(1);
  meas_cloud_.points[0].x = 0;
  meas_cloud_.points[0].y = 0;
  meas_cloud_.points[0].z = 0;

  // get parameters
  ros::NodeHandle local_nh("~");
  local_nh.param("fixed_frame", fixed_frame_, string("default"));
  local_nh.param("freq", freq_, 1.0);
  local_nh.param("start_distance_min", start_distance_min_, 0.0);
  local_nh.param("reliability_threshold", reliability_threshold_, 1.0);
  local_nh.param("sys_sigma_pos_x", sys_sigma_.pos_[0], 0.0);
  local_nh.param("sys_sigma_pos_y", sys_sigma_.pos_[1], 0.0);
  local_nh.param("sys_sigma_pos_z", sys_sigma_.pos_[2], 0.0);
  local_nh.param("sys_sigma_vel_x", sys_sigma_.vel_[0], 0.0);
  local_nh.param("sys_sigma_vel_y", sys_sigma_.vel_[1], 0.0);
  local_nh.param("sys_sigma_vel_z", sys_sigma_.vel_[2], 0.0);
  local_nh.param("follow_one_person", follow_one_person_, false);

  // advertise filter output
  people_filter_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_filter", 10);

  // advertise visualization
  people_filter_vis_pub_ = nh_.advertise<sensor_msgs::PointCloud>("people_tracker_filter_visualization", 10);
  people_tracker_vis_pub_ = nh_.advertise<sensor_msgs::PointCloud>("people_tracker_measurements_visualization", 10);

  // register message sequencer
  people_meas_sub_ = nh_.subscribe("people_tracker_measurements", 1, &PeopleTrackingNode::callbackRcv, this);

}


// destructor
PeopleTrackingNode::~PeopleTrackingNode()
{
  // delete sequencer
  delete message_sequencer_;

  // delete all trackers
  for (list<Tracker*>::iterator it = trackers_.begin(); it != trackers_.end(); it++)
    delete *it;
};




// callback for messages
void PeopleTrackingNode::callbackRcv(const people_msgs::PositionMeasurement::ConstPtr& message)
{
  ROS_DEBUG("Tracking node got a people position measurement (%f,%f,%f)",
            message->pos.x, message->pos.y, message->pos.z);
  // get measurement in fixed frame
  Stamped<tf::Vector3> meas_rel, meas;
  meas_rel.setData(
    tf::Vector3(message->pos.x, message->pos.y, message->pos.z));
  meas_rel.stamp_ = message->header.stamp;
  meas_rel.frame_id_ = message->header.frame_id;
  robot_state_.transformPoint(fixed_frame_, meas_rel, meas);

  // get measurement covariance
  SymmetricMatrix cov(3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      cov(i + 1, j + 1) = message->covariance[3 * i + j];

  // ----- LOCKED ------
  boost::mutex::scoped_lock lock(filter_mutex_);

  // update tracker if matching tracker found
  for (list<Tracker*>::iterator it = trackers_.begin(); it != trackers_.end(); it++)
    if ((*it)->getName() == message->object_id)
    {
      (*it)->updatePrediction(message->header.stamp.toSec());
      (*it)->updateCorrection(meas, cov);
    }
  // check if reliable message with no name should be a new tracker
  if (message->object_id == "" && message->reliability > reliability_threshold_)
  {
    double closest_tracker_dist = start_distance_min_;
    StatePosVel est;
    for (list<Tracker*>::iterator it = trackers_.begin(); it != trackers_.end(); it++)
    {
      (*it)->getEstimate(est);
      double dst = sqrt(pow(est.pos_[0] - meas[0], 2) + pow(est.pos_[1] - meas[1], 2));
      if (dst < closest_tracker_dist)
        closest_tracker_dist = dst;
    }
    // initialize a new tracker
    if (follow_one_person_)
      cout << "Following one person" << endl;
    if (message->initialization == 1 && ((!follow_one_person_ && (closest_tracker_dist >= start_distance_min_)) || (follow_one_person_ && trackers_.empty())))
    {
      //if (closest_tracker_dist >= start_distance_min_ || message->initialization == 1){
      //if (message->initialization == 1 && trackers_.empty()){
      ROS_INFO("Passed crazy conditional.");
      tf::Point pt;
      tf::pointMsgToTF(message->pos, pt);
      tf::Stamped<tf::Point> loc(pt, message->header.stamp, message->header.frame_id);
      robot_state_.transformPoint("base_link", loc, loc);
      float cur_dist;
      if ((cur_dist = pow(loc[0], 2.0) + pow(loc[1], 2.0)) < tracker_init_dist)
      {

        cout << "starting new tracker" << endl;
        stringstream tracker_name;
        StatePosVel prior_sigma(tf::Vector3(sqrt(cov(1, 1)), sqrt(cov(
                                              2, 2)), sqrt(cov(3, 3))), tf::Vector3(0.0000001, 0.0000001, 0.0000001));
        tracker_name << "person " << tracker_counter_++;
        Tracker* new_tracker = new TrackerKalman(tracker_name.str(),
            sys_sigma_);
        //Tracker* new_tracker = new TrackerParticle(tracker_name.str(), num_particles_tracker, sys_sigma_);
        new_tracker->initialize(meas, prior_sigma,
                                message->header.stamp.toSec());
        trackers_.push_back(new_tracker);
        ROS_INFO("Initialized new tracker %s", tracker_name.str().c_str());
      }
      else
        ROS_INFO("Found a person, but he/she is not close enough to start following.  Person is %f away, and must be less than %f away.", cur_dist , tracker_init_dist);
    }
    else
      ROS_INFO("Failed crazy conditional.");
  }
  lock.unlock();
  // ------ LOCKED ------


  // visualize measurement
  meas_cloud_.points[0].x = meas[0];
  meas_cloud_.points[0].y = meas[1];
  meas_cloud_.points[0].z = meas[2];
  meas_cloud_.header.frame_id = meas.frame_id_;
  people_tracker_vis_pub_.publish(meas_cloud_);
}



// callback for dropped messages
void PeopleTrackingNode::callbackDrop(const people_msgs::PositionMeasurement::ConstPtr& message)
{
  ROS_INFO("DROPPED PACKAGE for %s from %s with delay %f !!!!!!!!!!!",
           message->object_id.c_str(), message->name.c_str(), (ros::Time::now() - message->header.stamp).toSec());

}




// filter loop
void PeopleTrackingNode::spin()
{
  ROS_INFO("People tracking manager started.");

  while (ros::ok())
  {
    // ------ LOCKED ------
    boost::mutex::scoped_lock lock(filter_mutex_);

    // visualization variables
    vector<geometry_msgs::Point32> filter_visualize(trackers_.size());
    vector<float> weights(trackers_.size());
    sensor_msgs::ChannelFloat32 channel;

    // loop over trackers
    unsigned int i = 0;
    list<Tracker*>::iterator it = trackers_.begin();
    while (it != trackers_.end())
    {
      // update prediction up to delayed time
      (*it)->updatePrediction(ros::Time::now().toSec() - sequencer_delay);

      // publish filter result
      people_msgs::PositionMeasurement est_pos;
      (*it)->getEstimate(est_pos);
      est_pos.header.frame_id = fixed_frame_;

      ROS_DEBUG("Publishing people tracker filter.");
      people_filter_pub_.publish(est_pos);

      // visualize filter result
      filter_visualize[i].x = est_pos.pos.x;
      filter_visualize[i].y = est_pos.pos.y;
      filter_visualize[i].z = est_pos.pos.z;
      weights[i] = *(float*) & (rgb[min(998, 999 - max(1, (int)trunc((*it)->getQuality() * 999.0)))]);

      // remove trackers that have zero quality
      ROS_INFO("Quality of tracker %s = %f", (*it)->getName().c_str(), (*it)->getQuality());
      if ((*it)->getQuality() <= 0)
      {
        ROS_INFO("Removing tracker %s", (*it)->getName().c_str());
        delete *it;
        trackers_.erase(it++);
      }
      else it++;
      i++;
    }
    lock.unlock();
    // ------ LOCKED ------


    // visualize all trackers
    channel.name = "rgb";
    channel.values = weights;
    sensor_msgs::PointCloud  people_cloud;
    people_cloud.channels.push_back(channel);
    people_cloud.header.frame_id = fixed_frame_;
    people_cloud.points  = filter_visualize;
    people_filter_vis_pub_.publish(people_cloud);

    // sleep
    usleep(1e6 / freq_);

    ros::spinOnce();
  }
};


}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "people_tracker");
  ros::NodeHandle(nh);

  // create tracker node
  PeopleTrackingNode my_tracking_node(nh);

  // wait for filter to finish
  my_tracking_node.spin();

  // Clean up

  return 0;
}
