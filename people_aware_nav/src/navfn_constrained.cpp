/*********************************************************************
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <people_aware_nav/navfn_constrained.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_REGISTER_CLASS(NavfnROSConstrained, people_aware_nav::NavfnROSConstrained, nav_core::BaseGlobalPlanner);

namespace people_aware_nav {

using geometry_msgs::PoseStamped;
using std::vector;
using std::string;
using geometry_msgs::Point;
using geometry_msgs::Point32;

namespace vm=visualization_msgs;


NavfnROSConstrained::NavfnROSConstrained ()
{
  node_.param("hallway_global_frame_id", global_frame_, string("/map"));
}

NavfnROSConstrained::NavfnROSConstrained (string name, costmap_2d::Costmap2DROS* cmap)
{
  initialize(name, cmap);
}

void NavfnROSConstrained::initialize (string name, costmap_2d::Costmap2DROS* cmap)
{
  if(!initialized_){
  navfn::NavfnROS::initialize(name, cmap);
  ros::NodeHandle nh_private("~");
  service_ = nh_private.advertiseService("set_nav_constraint", &NavfnROSConstrained::setConstraint, this);
  vis_pub_add_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  }
  else
    ROS_WARN("You are attempting to initialize a planner that has already been initialized, doing nothing");
}

bool NavfnROSConstrained::setConstraint (SetNavConstraint::Request& req, SetNavConstraint::Response& resp)
{
  if(!initialized_){
    ROS_ERROR("This planner has not been initialized, please call initialize() before attempting to use it.");
    return false;
  }

  forbidden_ = req.forbidden;
  ROS_DEBUG_STREAM_NAMED("navfn", "Setting constraint polygon with " << forbidden_.points.size() << " points");
  return true;
}


void NavfnROSConstrained::getCostmap (costmap_2d::Costmap2D& cmap)
{
  if(!initialized_){
    ROS_ERROR("This planner has not been initialized, please call initialize() before attempting to use it.");
    return;
  }

  costmap_ros_->clearRobotFootprint();
  costmap_ros_->getCostmapCopy(cmap);

  // Set cost of forbidden region
  vector<Point> polygon;
  for (vector<Point32>::const_iterator iter = forbidden_.points.begin(); iter!=forbidden_.points.end(); ++iter) {
    Point p;
    p.x = iter->x;
    p.y = iter->y;
    p.z = iter->z;
    ROS_DEBUG_STREAM_NAMED("navfn", "Adding constraint point " << p.x << ", " << p.y << ", " << p.z);
    polygon.push_back(p);
  }
  cmap.setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);

  vm::Marker m;
  m.header.frame_id = global_frame_;
  m.header.stamp = ros::Time::now();
  m.ns = "pan";
  m.id = 0;
  m.type = vm::Marker::LINE_STRIP;
  m.action = vm::Marker::ADD;
  copy(polygon.begin(), polygon.end(), back_inserter(m.points));
  m.lifetime = ros::Duration(10.0);
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.a = 1.0;
  m.color.r = 1.0;
  vis_pub_add_.publish(m);
  

  ROS_DEBUG_NAMED ("navfn", "Modified costmap to take constraints into account");
}






} // namespace
