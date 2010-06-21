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

#include "timing_diagnostics_node.h"


using namespace std;


static const string node_name = "timing_diagnostics_node";


namespace estimation
{
  // constructor
  TimingDiagnosticsNode::TimingDiagnosticsNode()
    : ros::Node(node_name)
  {
    // subscribe to messages
    subscribe("people_tracker_measurements", measurement_, &TimingDiagnosticsNode::callbackMeasurement, 10);
    subscribe("people_tracker_filter", filter_, &TimingDiagnosticsNode::callbackFilter, 10);

    // open files for logging
    frontalface_file_.open("timing_frontalface.txt");
    profileface_file_.open("timing_profileface.txt");
    legtracker_file_.open("timing_legtracker.txt");
    filter_file_.open("timing_filter.txt");
  }




  // destructor
  TimingDiagnosticsNode::~TimingDiagnosticsNode()
  {
    frontalface_file_.close();
    profileface_file_.close();
    legtracker_file_.close();
  }




  // callback for messages
  void TimingDiagnosticsNode::callbackMeasurement()
  {
    //ROS_INFO("Timing Diagnostics: received measurement message from %s", measurement_.name.c_str());

    ros::Time time_now = ros::Time().now();

    if (measurement_.name == "face_detection_frontalface")
      frontalface_file_ << ros::Duration(time_now - measurement_.header.stamp).toSec() << " " << endl;

    if (measurement_.name == "face_detection_profileface")
      profileface_file_ << ros::Duration(time_now - measurement_.header.stamp).toSec() << " " << endl;

    if (measurement_.name == "leg_detector")
      legtracker_file_ << ros::Duration(time_now - measurement_.header.stamp).toSec() << " " << endl;
  }


  // callback for filter
  void TimingDiagnosticsNode::callbackFilter()
  {
    //ROS_INFO("Timing Diagnostics: received filter message from %s", filter_.name.c_str());

    ros::Time time_now = ros::Time().now();

    if (filter_.name == "")
      filter_file_ << ros::Duration(time_now - filter_.header.stamp).toSec() << " " << endl;
  }



}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv);

  // create timing diagnostics node
  TimingDiagnosticsNode my_timing_node;

  // wait for filter to finish
  my_timing_node.spin();

  // Clean up
  
  return 0;
}
