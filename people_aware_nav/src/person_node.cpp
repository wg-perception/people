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
#include <sensor_msgs/PointCloud.h>
#include <people_aware_nav/ConstrainedGoal.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace people_aware_nav
{

using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using people_msgs::PositionMeasurement;
using sensor_msgs::PointCloud;
using geometry_msgs::Pose2D;
using ros::Time;
using ros::Duration;
using tf::TransformException;
using std::string;

typedef tf::Stamped<tf::Pose> StampedPose;

namespace mf=message_filters;
typedef tf::MessageFilter<PointCloud> Filter;
typedef boost::shared_ptr<Filter> FilterPtr;
typedef mf::Subscriber<PointCloud> PCSub;
typedef boost::shared_ptr<PCSub> SubPtr;

double getYaw(const StampedPose& tf_pose)
{
  double pitch, roll, yaw;
  tf_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  return yaw;
}


// Transforms the hallway message and robot pose into the global_frame (usually map) and republishes them
class PersonPosSender
{
public:
  PersonPosSender()
  {

    nh_.param("hallway_global_frame_id", global_frame_, string("/map"));
    ROS_INFO_STREAM ("Person node has started, using global frame " << global_frame_);

    cloud_sub_ = SubPtr(new PCSub(nh_, "parallel_lines_model", 10));
    cloud_filter_ = FilterPtr(new Filter(*cloud_sub_, tf_, global_frame_, 50));
    cloud_filter_->registerCallback(boost::bind(&PersonPosSender::hallwayCallback, this, _1));
    
    hallway_points_pub_ = nh_.advertise<PointCloud>("hallway_points", 1);
    pose_pub_ = nh_.advertise<Pose2D>("robot_pose", 1);
  }


  void hallwayCallback (const PointCloud::ConstPtr& hallway_message)
  {
    ROS_DEBUG_NAMED ("person_node", "hallway callback");
    PointCloud msg;
    try {
      tf_.transformPointCloud(global_frame_, *hallway_message, msg);
      msg.header.stamp = ros::Time::now();
      hallway_points_pub_.publish(msg);
    }
    catch (TransformException& e)
    {
      ROS_INFO_STREAM ("TF exception transforming hallway " << e.what());
    }
  }

  void spin () 
  {
    Duration d(.5);

    while (ros::ok()) 
    {
      StampedPose id, odom_pose;
      id.setIdentity();
      d.sleep();
      ros::spinOnce();
      id.frame_id_ = "base_footprint";
      id.stamp_ = Time();
      try {
        tf_.transformPose(global_frame_, id, odom_pose);
        Pose2D pose;
        pose.x = odom_pose.getOrigin().x();
        pose.y = odom_pose.getOrigin().y();
        pose.theta = getYaw(odom_pose);

        pose_pub_.publish(pose);
      }
      catch (TransformException& e)
      {
        ROS_INFO_STREAM ("TF exception transforming odom pose " << e.what());
      }
    }
  }


private:
  ros::NodeHandle nh_;
  string global_frame_;
  people_msgs::PositionMeasurement person_message_;
  ConstrainedGoal goal_message_;
  PointCloud hallway_points_;
  tf::TransformListener tf_;
  SubPtr cloud_sub_;
  FilterPtr cloud_filter_;
  ros::Publisher hallway_points_pub_;
  ros::Publisher pose_pub_;

};
  
} // namespace


int main (int argc, char** argv)
{
  ros::init(argc, argv, "person_transform");
  people_aware_nav::PersonPosSender sender;
  sender.spin();
}


  

  
