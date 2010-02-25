/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/console.h>

#include <people_msgs/PositionMeasurement.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "people_aware_nav/LookStraightAhead.h"
#include "people_aware_nav/GlanceAt.h"
#include "people_aware_nav/StartHeadTrack.h"

namespace people_aware_nav
{

using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using std::string;

// A node that publishes head control messages for various types of high-level head actions.
class HeadController
{
public:
  HeadController() : private_nh_("~")
  {

    private_nh_.param("head_control_type", head_control_type_, string("look_forward"));
    private_nh_.param("default_speed", default_speed_, 1.0);
    // look_forward, glance, look_at_goal

    ROS_INFO_STREAM("Head exec type is " << head_control_type_);

    do_continuous_ = false;
    have_goal_ = false;

    point_head_pub_ = nh_.advertise<PointStamped>("/head_controller/point_head",1);

    look_ahead_service_ = nh_.advertiseService ("look_straight_ahead", &HeadController::lookStraightAhead, this);
    glance_service_ = nh_.advertiseService ("glance_at", &HeadController::glanceAt, this);
    head_track_service_ = nh_.advertiseService ("start_head_track", &HeadController::startHeadTrack, this);

    if (head_control_type_ == "look_forward") {
      do_continuous_ = false;
      state_ = STRAIGHT;
      lookStraight();
    }
    else if (head_control_type_ == "look_at_goal") {
      do_continuous_ = true;
      state_ = TRACK;
      goal_sub_ = nh_.subscribe("/hallway_move/goal", 10, &HeadController::goalCallback, this); //goal
    }

      
  }

  // Service callbacks
  bool lookStraightAhead(people_aware_nav::LookStraightAhead::Request &req, people_aware_nav::LookStraightAhead::Response& resp) {

    do_continuous_ = false;
    state_ = STRAIGHT;
    lookStraight();
    return true;
  }

  bool glanceAt(people_aware_nav::GlanceAt::Request &req, people_aware_nav::GlanceAt::Response& resp) {

    do_continuous_ = false; // Turn off track point publishing
    point_head_pub_.publish(req.point_stamped);
    usleep(2000000);
    if (state_== TRACK) {
      lookAtGoal(); // Look at the goal once with a reasonable speed before passing it back to the continuous publication.
      do_continuous_ = true;
    }
    else {
      lookStraight();
    }
    return true;
    
  }
 
  bool startHeadTrack(people_aware_nav::StartHeadTrack::Request &req, people_aware_nav::StartHeadTrack::Response& resp) {
    do_continuous_ = true;
    state_ = TRACK;
    // head_goal_sub_ = nh_.subscribe("goal", &HeadController::goalCallback, this, 1); //goal                                                                              
    return true;
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_message) 
  {
    goal_pose_= *goal_message;
    have_goal_ = true;
  }

  void spin () 
  {
    
    ros::Duration d(.1);
    while (ros::ok()) 
    {
      if (do_continuous_ && have_goal_)
      {
	lookAtGoal(-1);
        d.sleep();
      }
      ros::spin();
    }
  }


private:
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher point_head_pub_;
  ros::ServiceServer look_ahead_service_, glance_service_, head_track_service_;
  ros::Subscriber goal_sub_, head_goal_sub_;

  string head_control_type_;
  double default_speed_;
  geometry_msgs::PoseStamped goal_pose_;
  bool do_continuous_, have_goal_;

  enum {NONE=0, TRACK=1, STRAIGHT=2};
  int state_;

  void lookStraight() {
    lookStraight(default_speed_);
  }
 
  void lookStraight( double speed) {
    if (speed < 0.0) {
      // As fast as possible
      PointStamped p;
      p.header.frame_id = "base_link";
      p.header.stamp = ros::Time::now();
      p.point.x = 1;
      p.point.y = 0;
      p.point.z = 1.1;
      point_head_pub_.publish(p);
    }
    else {
      // Change this to include the speed.
      PointStamped p;
      p.header.frame_id = "base_link";
      p.header.stamp = ros::Time::now();
      p.point.x = 1;
      p.point.y = 0;
      p.point.z = 1.1;
      point_head_pub_.publish(p);
    }
  }

  void lookAtGoal() {
    lookAtGoal(default_speed_);
  }

  void lookAtGoal(double speed) 
  {
    if (speed < 0.0) {
      // As fast as possible
      PointStamped p;
      p.header.frame_id = goal_pose_.header.frame_id;
      p.header.stamp = ros::Time::now();
      p.point = goal_pose_.pose.position;
      // BMM: Commenting this and the next publish out during move to nodehandle
      // as there's no corresponding advertise
      // node_.publish("/head_controller/head_track_point",p);
    }
    else {
      // Change this to include the speed.
      PointStamped p;
      p.header.frame_id = goal_pose_.header.frame_id;
      p.header.stamp = ros::Time::now();
      p.point = goal_pose_.pose.position;
      // node_.publish("/head_controller/head_track_point",p);
    }
  }

}; // class
  
} // namespace


int main (int argc, char** argv)
{
  ros::init(argc, argv, "head_controller");
  people_aware_nav::HeadController hc;
  hc.spin();
}


  

  
