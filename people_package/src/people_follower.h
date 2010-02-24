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

#ifndef __PEOPLE_FOLLOWER__
#define __PEOPLE_FOLLOWER__

#include <string>
#include <boost/thread/mutex.hpp>

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>

// messages
#include <sensor_msgs/PointCloud.h>
#include <people_package/PositionMeasurement.h>
#include <pr2_robot_actions/Pose2D.h>
 
// log files
#include <fstream>

namespace estimation
{

class PeopleFollower
{
public:
  /// constructor
  PeopleFollower(const std::string& node_name_);

  /// destructor
  ~PeopleFollower();

  /// callback for people messages
  void callback(const tf::MessageNotifier<people_package::PositionMeasurement>::MessagePtr& people_pos);


private:
  // Viz marker
  void visualizeGoalPose(const geometry_msgs::PoseStamped& poseStamped);
  void visualizePeoplePose(const pr2_robot_actions::Pose2D &pos1, const pr2_robot_actions::Pose2D &pos2);

  //Callback for second thread sending periodic goals and head positions
  void sendGoalAndHeadPosition();

  // name
  std::string node_name_, fixed_frame_;

  // tf listener
  tf::TransformListener robot_state_;
  tf::TransformBroadcaster tf_server_;

  // message notifier
  tf::MessageNotifier<people_package::PositionMeasurement>*  people_notifier_;

  // list of distances and people poses
  std::list<double> distances_;
  std::list<pr2_robot_actions::Pose2D> people_poses_;

  // message
  pr2_robot_actions::Pose2D people_pos_, robot_pos_;

  ros::Time time_last_publish_;

  double follow_distance_, publish_rate_, distance_threshold_;
  bool initialized_;
  sensor_msgs::PointCloud  robot_goal_cloud_; 

  ros::NodeHandle node_;
  ros::Publisher filter_output_pub_;
  ros::Publisher viz_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher head_target_pub_;

  std::ofstream foutRobot;
  std::ofstream foutPerson;
  std::ofstream foutGoal;

  boost::mutex head_pos_mutex_;
  geometry_msgs::PointStamped last_head_pos_;
  bool head_pos_received_;
  boost::mutex goal_pos_mutex_;
  geometry_msgs::PoseStamped last_goal_pos_;
  bool goal_pos_received_;

  friend struct processingThread;
  struct processingThread
  {
    PeopleFollower *parent;

    processingThread(PeopleFollower *parent)
    {
      this->parent = parent;
    }

    void operator()()
    { //The function that runs when the thread is started
      while(parent->node_.ok())
        parent->sendGoalAndHeadPosition();
    }
  };

}; // class

}; // namespace

#endif
