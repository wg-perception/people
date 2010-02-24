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
#ifndef PEOPLE_AWARE_NAV_MOVE_BASE_ACTION_H_
#define PEOPLE_AWARE_NAV_MOVE_BASE_ACTION_H_
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <people_aware_nav/ConstrainedMoveBaseState.h>
#include <people_aware_nav/ConstrainedGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <vector>
#include <string>
#include <nav_msgs/GetPlan.h>

namespace people_aware_nav {
  /**
   * @class MoveBaseConstrained
   * @brief Like nav::MoveBase, but allows constraints on where robot can move
   */
  class MoveBaseConstrained : public robot_actions::Action<ConstrainedGoal, geometry_msgs::PoseStamped> {
    public:
      /**
       * @brief  Constructor for the actions
       * @param ros_node A reference to the ros node used 
       * @param tf A reference to a TransformListener
       * @return 
       */
      MoveBaseConstrained(ros::NodeHandle ros_node, tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBaseConstrained();

      /**
       * @brief  Runs whenever a new goal is sent to the move_base
       * @param goal The goal to pursue 
       * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
       * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
       */
      virtual robot_actions::ResultStatus execute(const ConstrainedGoal& goal, geometry_msgs::PoseStamped& feedback);

    private:
      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       */
    void makePlan(const geometry_msgs::PoseStamped& goal, const geometry_msgs::Polygon& forbidden);

      /**
       * @brief  Get the current pose of the robot in the specified frame
       * @param  frame The frame to get the pose in
       * @param  pose The pose returned
       */
      void getRobotPose(std::string frame, tf::Stamped<tf::Pose>& pose);

      /**
       * @brief  Resets the costmaps to the static map outside a given window
       */
      void resetCostmaps(double size_x, double size_y);

      void resetState();

    bool tryPlan(geometry_msgs::PoseStamped goal, const geometry_msgs::Polygon& forbidden);

      ros::NodeHandle ros_node_;
      tf::TransformListener& tf_;
      bool run_planner_;
      base_local_planner::TrajectoryPlannerROS* tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;
      costmap_2d::Costmap2D planner_costmap_, controller_costmap_;

      navfn::NavfnROS* planner_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      std::vector<geometry_msgs::Point> footprint_;
      std::string robot_base_frame_, global_frame_;
      bool valid_plan_, new_plan_;
      boost::recursive_mutex lock_;
      geometry_msgs::PoseStamped goal_;

      tf::Stamped<tf::Pose> global_pose_;
      double controller_frequency_, inscribed_radius_, circumscribed_radius_, planner_patience_, controller_patience_;
      bool attempted_rotation_, attempted_costmap_reset_;
      bool done_half_rotation_, done_full_rotation_;
      ros::Time last_valid_control_;
      ros::Publisher cmd_vel_pub_;

  };
};
#endif

