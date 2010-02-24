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

#include "people_follower.h"

#include <visualization_msgs/Marker.h>

using namespace std;
using namespace ros;
using namespace tf;

namespace estimation
{
  // constructor
  PeopleFollower::PeopleFollower(const string& node_name)
    : node_name_(node_name),
      people_notifier_(NULL),
      initialized_(false),
      foutRobot("/u/ethand/RobotPts.txt"),
      foutPerson("/u/ethand/PersonPts.txt"),
      foutGoal("/u/ethand/GoalPts.txt")
  {
    // get parameters
	node_.param("~/follow_distance", follow_distance_, 1.25);
	node_.param("~/distance_threshold", distance_threshold_, 0.1);
	node_.param("~/fixed_frame", fixed_frame_, string("/odom_combined"));
	fixed_frame_ = "/odom_combined";
	cout<<" *** Fixed frame is: "<<fixed_frame_<<endl;
	node_.param("~/publish_rate", publish_rate_, 2.0);

    // advertise filter output
	filter_output_pub_ = node_.advertise<people_package::PositionMeasurement>("people_tracker_filter",10);

    // advertise visualization
	viz_pub_ = node_.advertise<sensor_msgs::PointCloud>("goal_pos",10);

    // register message sequencer
    people_notifier_ = new MessageNotifier<people_package::PositionMeasurement>(robot_state_, boost::bind(&PeopleFollower::callback, this, _1),
                                                               "people_tracker_filter", fixed_frame_, 10);
    // advertise robot poses
    goal_pub_ = node_.advertise<geometry_msgs::PoseStamped>("/move_base_local/activate", 10);

    marker_pub_ = node_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    head_target_pub_ = node_.advertise<geometry_msgs::PointStamped>("/head_controller/point_head",1);

    // initialize goal
    people_pos_.header.frame_id = fixed_frame_;
    people_pos_.header.stamp = ros::Time().fromSec(0);

    robot_pos_.header.frame_id = fixed_frame_;
    robot_pos_.header.stamp = ros::Time().fromSec(0);

    time_last_publish_ = Time::now();

    // visualization
    robot_goal_cloud_.points = vector<geometry_msgs::Point32>(1);
    robot_goal_cloud_.points[0].x = 0;
    robot_goal_cloud_.points[0].y = 0;
    robot_goal_cloud_.points[0].z = 0;

    head_pos_received_ = false;
    goal_pos_received_ = false;
    boost::thread(processingThread(this));
  }

  // destructor
  PeopleFollower::~PeopleFollower()
  {
    delete people_notifier_;
  };

  void PeopleFollower::sendGoalAndHeadPosition()
  {
    static ros::Rate rate_limit(2.0);
    rate_limit.reset();

    //Lock head pos and send
    if(head_pos_received_)
    {
      boost::mutex::scoped_lock lock(head_pos_mutex_);
      head_target_pub_.publish(last_head_pos_);
    }

    //Lock goal pos and send
    if(goal_pos_received_)
    {
      boost::mutex::scoped_lock lock(goal_pos_mutex_);
      goal_pub_.publish(last_goal_pos_);
    }

    //delay to hit desired rate
    rate_limit.sleep();
  }

  void PeopleFollower::visualizeGoalPose(const geometry_msgs::PoseStamped& poseStamped)
  {
    static int trail_counter_ = 0;
    static bool first_viz_time_ = true;
    static geometry_msgs::PoseStamped ps_;
    if(first_viz_time_)
      {
	first_viz_time_ = false;
	ps_ = poseStamped;
	return;
      }
    
    visualization_msgs::Marker marker;
    marker.header = poseStamped.header;
    //marker.header.frame_id = fixed_frame_;
    //    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_trail";
    marker.id = trail_counter_;
    trail_counter_++;
    //if(trail_counter_ > 4) trail_counter_ = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD; //Add and update are the same enum
    geometry_msgs::Point p;
    p.x = poseStamped.pose.position.x;
    p.y = poseStamped.pose.position.y;
    p.z = 0.05;
    marker.points.push_back(p);
    p.x = ps_.pose.position.x;
    p.y = ps_.pose.position.y;
    p.z = 0.06;
    marker.points.push_back(p);
    marker.scale.x = 0.30;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;//0.0 + 0.9 * ((double)trail_counter_ / 4.0);
    marker.color.g = 0.7;//0.9 - 0.9 * ((double)trail_counter_ / 4.0);;
    marker.color.b = 0.0;
    
    ps_ = poseStamped;

    marker_pub_.publish( marker );

    try {
      geometry_msgs::PointStamped zero_point, map_point;
      zero_point.header = poseStamped.header;
      zero_point.point.x = poseStamped.pose.position.x;
      zero_point.point.y = poseStamped.pose.position.y;
      zero_point.point.z = 0.0;
      std::string error_msg;
      ros::Time current_time = ros::Time::now();
      ros::Duration timeout(0.5);
      robot_state_.waitForTransform("map",poseStamped.header.frame_id, current_time,timeout,ros::Duration(0.01),&error_msg);
      robot_state_.transformPoint("map", zero_point, map_point);
      foutGoal<<map_point.point.x<<" "<<map_point.point.y<<endl;
    } catch (...) { cout<<"Couldn't find goal position in map."<<endl; }



    /*
      cout<<"Sending vizualization message!"<<endl;
    visualization_msgs::Marker marker;
    marker.header = poseStamped.header;
    marker.ns = "follower";
    marker.id = trail_counter_;
    trail_counter_++;
    if(trail_counter_ > 25) trail_counter_ = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD; //Add and update are the same enum
    marker.pose = poseStamped.pose;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.7;
    marker.color.r = 0.0 + 0.5 * ((double)trail_counter_ / 25.0);
    marker.color.g = 0.8 - 0.5 * ((double)trail_counter_ / 25.0);;
    marker.color.b = 0.3;

    ROS_INFO("setting viz: Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)\n",
	     marker.pose.position.x,
	     marker.pose.position.y,
	     marker.pose.position.z,
	     marker.pose.orientation.x,
	     marker.pose.orientation.y,
	     marker.pose.orientation.z,
	     marker.pose.orientation.w);

    marker_pub_.publish( marker );
    */
  }

  void PeopleFollower::visualizePeoplePose(const pr2_robot_actions::Pose2D &pos1, const pr2_robot_actions::Pose2D &pos2)
  {
    static int trail_counter_ = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "person_trail";
    marker.id = trail_counter_;
    trail_counter_++;
    //if(trail_counter_ > 4) trail_counter_ = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD; //Add and update are the same enum
    geometry_msgs::Point p;
    p.x = pos1.x;
    p.y = pos1.y;
    p.z = 0.05;
    marker.points.push_back(p);
    p.x = pos2.x;
    p.y = pos2.y;
    p.z = 0.05;
    marker.points.push_back(p);
    marker.scale.x = 0.15;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;//0.0 + 0.9 * ((double)trail_counter_ / 4.0);
    marker.color.g = 0.0;//0.9 - 0.9 * ((double)trail_counter_ / 4.0);;
    marker.color.b = 0.6;

    ROS_INFO("setting viz: Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)\n",
	     marker.pose.position.x,
	     marker.pose.position.y,
	     marker.pose.position.z,
	     marker.pose.orientation.x,
	     marker.pose.orientation.y,
	     marker.pose.orientation.z,
	     marker.pose.orientation.w);

    try {
      geometry_msgs::PointStamped zero_point, map_point;
      zero_point.header = marker.header;
      zero_point.point.x = p.x;
      zero_point.point.y = p.y;
      zero_point.point.z = 0.0;
      std::string error_msg;
      ros::Time current_time = ros::Time::now();
      ros::Duration timeout(0.5);
      robot_state_.waitForTransform("map",marker.header.frame_id, current_time,timeout,ros::Duration(0.01),&error_msg);
      robot_state_.transformPoint("map", zero_point, map_point);
      foutPerson<<map_point.point.x<<" "<<map_point.point.y<<endl;
    } catch (...) { cout<<"Couldn't find goal position in map."<<endl; }

    //    foutPerson<<p.x<<" "<<p.y<<endl;

    marker_pub_.publish( marker );
    
    static int trail_counter2_ = 0;
    geometry_msgs::PointStamped zero_point;
    static geometry_msgs::PointStamped map_point;
    try {      
      zero_point.header.frame_id = "base_link";
      zero_point.header.stamp = ros::Time::now();
      zero_point.point.x = 0.0;
      zero_point.point.y = 0.0;
      zero_point.point.z = 0.0;
      std::string error_msg;
      ros::Time current_time = ros::Time::now();
      ros::Duration timeout(0.5);
      robot_state_.waitForTransform("map","base_link", current_time,timeout,ros::Duration(0.01),&error_msg);
      robot_state_.transformPoint("map", zero_point, map_point);
    } catch (...) { cout<<"Couldn't find robot position."<<endl; }

    static bool firstTimeThrough = true;
    static geometry_msgs::PointStamped last_point;
    if(firstTimeThrough)
      {
	firstTimeThrough = false;
	last_point = map_point;
	return;
      }

    marker.id = trail_counter2_++;
    marker.ns = "robot_trail";
    marker.header.frame_id = "map";
    marker.points.clear();
    p.x = last_point.point.x;
    p.y = last_point.point.y;
    p.z = 0.05;
    cout<<" **** point 1: "<<p.x<<","<<p.y<<endl;
    marker.points.push_back(p);
    p.x = map_point.point.x;
    p.y = map_point.point.y;
    p.z = 0.05;
    cout<<" **** point 1: "<<p.x<<","<<p.y<<endl;
    marker.points.push_back(p);
    marker.color.r = 0.8;//0.0 + 0.9 * ((double)trail_counter_ / 4.0);                                                                                            
    marker.color.g = 0.0;//0.9 - 0.9 * ((double)trail_counter_ / 4.0);;                                                                                           
    marker.color.b = 0.0;
    marker_pub_.publish( marker );
    last_point = map_point;

    foutRobot<<p.x<<" "<<p.y<<endl;
    
  }

  // callback for messages
  void PeopleFollower::callback(const MessageNotifier<people_package::PositionMeasurement>::MessagePtr& people_pos_msg)
  {
	ROS_DEBUG("Got a people position measurement callback!");
	cout<<"Got a callback people position msg, almost there!"<<endl;
 
    // get people pos in fixed frame
    Stamped<tf::Vector3> people_pos_rel, people_pos_fixed_frame;
    people_pos_rel.setData(tf::Vector3(people_pos_msg->pos.x, people_pos_msg->pos.y, people_pos_msg->pos.z));
    people_pos_rel.stamp_    = people_pos_msg->header.stamp;
    people_pos_rel.frame_id_ = people_pos_msg->header.frame_id;
    
    try {
    robot_state_.transformPoint(fixed_frame_, people_pos_rel, people_pos_fixed_frame);
    } catch(...) { ROS_WARN("People follower tf error."); }

    // convert to planner 2D goal message
    people_pos_.x = people_pos_fixed_frame.x();
    people_pos_.y = people_pos_fixed_frame.y();
    people_pos_.th = 0.0;

    if (initialized_){
      // calculate distance and angle
      double dx = people_pos_.x - people_poses_.back().x;
      double dy = people_pos_.y - people_poses_.back().y;
      double length = sqrt(pow(dx,2) + pow(dy,2));
      people_pos_.th = atan2(dy, dx);

      // add to list buffer 
      if (length > distance_threshold_){
        distances_.push_back( distances_.back() + length);	
	if(people_poses_.size() > 0)
	  visualizePeoplePose(people_poses_.back(), people_pos_);
        people_poses_.push_back(people_pos_);

        static tf::StampedTransform transform = tf::StampedTransform(btTransform(btQuaternion(0,0,0), btVector3(people_pos_.x,people_pos_.y,0.0)), ros::Time::now(), fixed_frame_, "/CURRENT_PERSON_POS" );
	transform.stamp_ = ros::Time::now();//people_pos_.header.stamp;
	tf_server_.sendTransform(transform);
	cout<<"     =========    Sending tf transform: "<<people_pos_.x<<","<<people_pos_.y<<","<<0.0<<" in "<<fixed_frame_<<" at "<<transform.stamp_<<endl;

        geometry_msgs::PointStamped head_target;
	head_target.point.x = people_pos_.x; 
	head_target.point.y = people_pos_.y; 
	head_target.point.z = 1.2; 
	head_target.header = people_pos_.header;
	head_target.header.stamp = ros::Time::now();
	//cout<<"Publishing head target: "<<head_target.point.x<<","<<head_target.point.y<<","<<head_target.point.z<<" in frame: "<<head_target.header.frame_id<<endl;
	{
          boost::mutex::scoped_lock lock(head_pos_mutex_);
          last_head_pos_ = head_target;
          head_pos_received_ = true;
        }
      }

      // find next goal to send, which is follow_distance_ away from people_pos_
      while  (sqrt(pow(people_pos_.x-people_poses_.front().x,2) +
                   pow(people_pos_.y-people_poses_.front().y,2) ) > follow_distance_) {
        people_poses_.pop_front();
        distances_.pop_front();
      }
      robot_pos_ = people_poses_.front();

      Stamped<tf::Vector3> robot_in_fixed_frame, zeroPoint;
      zeroPoint.stamp_ = people_pos_msg->header.stamp;
      zeroPoint.frame_id_ = "/base_link";
      zeroPoint.setData(tf::Vector3(0.0, 0.0, 0.0));
      try {	
	robot_state_.transformPoint(fixed_frame_, zeroPoint, robot_in_fixed_frame);
      } catch(...) { ROS_WARN("People follower tf error getting current robot location."); }

      dx = robot_pos_.x - robot_in_fixed_frame.x();
      dy = robot_pos_.y - robot_in_fixed_frame.y();
      if(people_poses_.size() > 0)
	{
	  dx = people_poses_.front().x - robot_in_fixed_frame.x();
	  dy = people_poses_.front().y - robot_in_fixed_frame.y();
	}
      robot_pos_.th = atan2(dy, dx);

      /*
      cout << "person pos " 
           << people_pos_.goal.x << " "  
           << people_pos_.goal.y << " " 
           << people_pos_.goal.th << endl;
      cout << "robot  pos " 
           << robot_pos_.goal.x << " "
           << robot_pos_.goal.y << " "
           << robot_pos_.goal.th << endl;
      cout << "distance between them "<< sqrt(pow(people_pos_.goal.x-robot_pos_.goal.x,2) +
                                                    pow(people_pos_.goal.y-robot_pos_.goal.y,2) ) << endl;
      */

      double timeElapsed;
      // send goal to planner
      //if ((timeElapsed = (Time::now() - time_last_publish_).toSec()) > 1.0/publish_rate_){
      //ROS_INFO("Enough time has passed: %f seconds.", timeElapsed);
        //publish("goal", robot_pos_);
	    geometry_msgs::PoseStamped goal_pos_;
        goal_pos_.header = people_pos_.header;
        goal_pos_.header.frame_id = "/odom_combined";
        goal_pos_.header.stamp = Time::now();
        goal_pos_.pose.position.x = robot_pos_.x; //people_pose_ or robot_pos_?
        goal_pos_.pose.position.y = robot_pos_.y;
        goal_pos_.pose.position.z = robot_pos_.z;
        tf::Quaternion quat(tf::Vector3(0.0,0.0,1.0),robot_pos_.th); //TODO: sort out robot_pos_ vs people_pos_ (i.e. why is robot_pos_.th set to the desired angle?)
        goal_pos_.pose.orientation.x = quat.x();
        goal_pos_.pose.orientation.y = quat.y();
        goal_pos_.pose.orientation.z = quat.z();
        goal_pos_.pose.orientation.w = quat.w();
	
        static tf::StampedTransform transform = tf::StampedTransform(btTransform(/*goal_pos_.pose.orientation*/btQuaternion(0,0,0), btVector3(goal_pos_.pose.position.x, goal_pos_.pose.position.y, 0.0)), ros::Time::now() , goal_pos_.header.frame_id, "/CURRENT_GOAL_POS");
	transform.stamp_ = people_pos_.header.stamp;//ros::Time::now();
	tf_server_.sendTransform(transform);

        visualizeGoalPose( goal_pos_ );

        //Lock goal pos and update
        {
          boost::mutex::scoped_lock lock(goal_pos_mutex_);
          last_goal_pos_ = goal_pos_;
          goal_pos_received_ = true;
        }
        //goal_pub_.publish( goal_pos_);

        time_last_publish_ = Time::now();
      /*}
      else
      {
    	  ROS_INFO("Only %f seconds have passed.", timeElapsed);
      }*/

      // visualize goal
      robot_goal_cloud_.points[0].x = robot_pos_.x;
      robot_goal_cloud_.points[0].y = robot_pos_.y;
      robot_goal_cloud_.points[0].z = 0.0;
      robot_goal_cloud_.header.frame_id = fixed_frame_;
      viz_pub_.publish(robot_goal_cloud_);
    }
    

    // not initialized yet
    else{
      cout << "Initializing" << endl;
      people_poses_.push_back(people_pos_);
      distances_.push_back(0.0);
      initialized_ = true;
    }
  }
} // namespace



// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  std::string node_name = "people_follower";

  // Initialize ROS
  ros::init(argc, argv, node_name);

  // create people follower
  PeopleFollower my_people_follower(node_name);

  // wait for filter to finish
  ros::spin();

  return 0;
}
