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
#include "ros/ros.h"

#include "laser_processor.h"
#include "calc_leg_features.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "rosrecord/Player.h"

#include "people_msgs/PositionMeasurement.h"
#include "sensor_msgs/LaserScan.h"
#include "roslib/Header.h"

#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "people_tracking_filter/tracker_kalman.h"
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"

#include <algorithm>

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;


static const double no_observation_timeout_s = 0.5;
static const double max_second_leg_age_s     = 2.0;
static const double max_track_jump_m         = 1.0; 
static const double max_meas_jump_m          = 0.75; // 1.0
static const double leg_pair_separation_m    = 1.0;
static const string fixed_frame              = "odom_combined";


class SavedFeature
{
public:
  static int nextid;
  TransformListener& tfl_;

  BFL::StatePosVel sys_sigma_;
  TrackerKalman filter_;

  string id_;
  string object_id;
  ros::Time time_;
  ros::Time meas_time_;

  Stamped<Point> position_;
  float dist_to_person_;

  // one leg tracker
  SavedFeature(Stamped<Point> loc, TransformListener& tfl)
    : tfl_(tfl),
      sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
      filter_("tracker_name",sys_sigma_)
  {
    char id[100];
    snprintf(id,100,"legtrack%d", nextid++);
    id_ = std::string(id);

    object_id = "";
    time_ = loc.stamp_;
    meas_time_ = loc.stamp_;

    try {
    tfl_.transformPoint(fixed_frame, loc, loc);
    } catch(...) {
      ROS_WARN("TF exception spot 6.");
    }
    StampedTransform pose( btTransform(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
    tfl_.setTransform(pose);

    StatePosVel prior_sigma(Vector3(0.1,0.1,0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
    filter_.initialize(loc, prior_sigma, time_.toSec());    

    StatePosVel est;
    filter_.getEstimate(est);

    updatePosition();
  }

  void propagate(ros::Time time)
  {
    time_ = time;

    filter_.updatePrediction(time.toSec());

    updatePosition();
  }

  void update(Stamped<Point> loc)
  {
    StampedTransform pose( btTransform(Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
    tfl_.setTransform(pose);

    meas_time_ = loc.stamp_;
    time_ = meas_time_;

    SymmetricMatrix cov(3);
    cov = 0.0;
    cov(1,1) = 0.0025;
    cov(2,2) = 0.0025;
    cov(3,3) = 0.0025;

    filter_.updateCorrection(loc, cov);

    updatePosition();
  }

  double getLifetime()
  {
    return filter_.getLifetime();
  }

private:
  void updatePosition()
  {
    StatePosVel est;
    filter_.getEstimate(est);

    position_[0] = est.pos_[0];
    position_[1] = est.pos_[1];
    position_[2] = est.pos_[2];
    position_.stamp_ = time_;
    position_.frame_id_ = fixed_frame;
  }
};

int SavedFeature::nextid = 0;




class MatchedFeature
{
public:
  SampleSet* candidate_;
  SavedFeature* closest_;
  float distance_;

  MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance)
  : candidate_(candidate)
  , closest_(closest)
  , distance_(distance)
  {}

  inline bool operator< (const MatchedFeature& b) const
  {
    return (distance_ <  b.distance_);
  }
};

int g_argc;
char** g_argv;




// actual legdetector node
class LegDetector
{
public:
  NodeHandle nh_;

  TransformListener tfl_;

  ScanMask mask_;

  int mask_count_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  char save_[100];

  list<SavedFeature*> saved_features_;
  boost::mutex saved_mutex_;

  int feature_id_;

  ros::Publisher leg_cloud_pub_;
  ros::Publisher tracker_measurements_pub_;

  message_filters::Subscriber<people_msgs::PositionMeasurement> people_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<people_msgs::PositionMeasurement> people_notifier_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

  LegDetector(ros::NodeHandle nh) :
    nh_(nh), 
    mask_count_(0), 
    connected_thresh_(0.06), 
    feat_count_(0),
    people_sub_(nh_,"people_tracker_filter",10),
    laser_sub_(nh_,"scan",10),
    people_notifier_(people_sub_,tfl_,fixed_frame,10),
    laser_notifier_(laser_sub_,tfl_,fixed_frame,10)
  {
    if (g_argc > 1) {
      forest.load(g_argv[1]);
      feat_count_ = forest.get_active_var_mask()->cols;
      printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
    } else {
      printf("Please provide a trained random forests classifier as an input.\n");
      shutdown();
    }

    // advertise topics
    leg_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("kalman_filt_cloud",10);
    tracker_measurements_pub_ = nh_.advertise<people_msgs::PositionMeasurement>("people_tracker_measurements",1);

    people_notifier_.registerCallback(boost::bind(&LegDetector::peopleCallback, this, _1));
    people_notifier_.setTolerance(ros::Duration(0.01));
    laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));

    feature_id_ = 0;
  }


  ~LegDetector()
  {
  }



  // Find the tracker that is closest to this person message
  // If a tracker was already assigned to a person, keep this assignment when the distance between them is not too large.
  void peopleCallback(const people_msgs::PositionMeasurement::ConstPtr& people_meas)
  {
    // If there are no legs, return.
    if (saved_features_.empty()) 
      return;

    Point pt;
    pointMsgToTF(people_meas->pos, pt);
    Stamped<Point> person_loc(pt, people_meas->header.stamp, people_meas->header.frame_id);
    person_loc[2] = 0.0; // Ignore the height of the person measurement.
    Stamped<Point> dest_loc(pt, people_meas->header.stamp, people_meas->header.frame_id); // Holder for all transformed pts.

    boost::mutex::scoped_lock lock(saved_mutex_);

    list<SavedFeature*>::iterator closest = saved_features_.end();
    list<SavedFeature*>::iterator closest1 = saved_features_.end();
    list<SavedFeature*>::iterator closest2 = saved_features_.end();
    float closest_dist = max_meas_jump_m;
    float closest_pair_dist = 2*max_meas_jump_m;

    list<SavedFeature*>::iterator begin = saved_features_.begin();
    list<SavedFeature*>::iterator end = saved_features_.end();
    list<SavedFeature*>::iterator it1, it2;

    // If there's a pair of legs with the right label and within the max dist, return
    // If there's one leg with the right label and within the max dist, 
    //   find a partner for it from the unlabeled legs whose tracks are reasonably new.
    //   If no partners exist, label just the one leg.
    // If there are no legs with the right label and within the max dist,
    //   find a pair of unlabeled legs and assign them the label.
    // If all of the above cases fail, 
    //   find a new unlabeled leg and assign the label.
    
    // For each tracker, get the distance to this person.
    for (it1 = begin; it1 != end; ++it1) 
    {
      try {
      tfl_.transformPoint((*it1)->id_, people_meas->header.stamp,
			  person_loc, fixed_frame, dest_loc);
      //ROS_INFO("Succesful leg transformation at spot 7"); 
      } catch(...) {
        ROS_WARN("TF exception spot 7.");
      }
      (*it1)->dist_to_person_ = dest_loc.length();
    }

    // Try to find one or two trackers with the same label and within the max distance of the person.
    cout << "Looking for two legs" << endl;
    it2 = end;
    for (it1 = begin; it1 != end; ++it1)
    {
      // If this leg belongs to the person...
      if ((*it1)->object_id == people_meas->object_id) 
      {
	// and their distance is close enough... 
	if ((*it1)->dist_to_person_ < max_meas_jump_m)
	{
	  // if this is the first leg we've found, assign it to it2. Otherwise, leave it assigned to it1 and break.
	  if (it2 == end) 
	    it2 = it1;
	  else
	    break;
	}
	// Otherwise, remove the tracker's label, it doesn't belong to this person.
	else{
          // the two trackers moved apart. This should not happen.
	  (*it1)->object_id = "";
        }
      }
    }
    // If we found two legs with the right label and within the max distance, all is good, return.
    if (it1 != end && it2 != end) 
    {
      cout << "Found matching pair. The second distance was " << (*it1)->dist_to_person_ << endl;
      return;
    }



    // If we only found one close leg with the right label, let's try to find a second leg that 
    //   * doesn't yet have a label  (=valid precondition),
    //   * is within the max distance,
    //   * is less than max_second_leg_age_s old.
    cout << "Looking for one leg plus one new leg" << endl;
    float dist_between_legs, closest_dist_between_legs;
    if (it2 != end) 
    {
      closest_dist = max_meas_jump_m;
      closest = saved_features_.end();

      for (it1 = begin; it1 != end; ++it1) 
      {
	// Skip this leg track if:
	// - you're already using it.
	// - it already has an id.
	// - it's too old. Old unassigned trackers are unlikely to be the second leg in a pair.
	// - it's too far away from the person.
	if ((it1 == it2) || ((*it1)->object_id != "") ||  ((*it1)->getLifetime() > max_second_leg_age_s) || ((*it1)->dist_to_person_ >= closest_dist) )
	  continue;
	
	// Get the distance between the two legs
	try {
		tfl_.transformPoint((*it1)->id_, (*it2)->position_.stamp_, (*it2)->position_, fixed_frame, dest_loc);
	} catch(...) {
		ROS_WARN("TF exception getting distance between legs.");
	}
	dist_between_legs = dest_loc.length();

	// If this is the closest dist (and within range), and the legs are close together and unlabeled, mark it.
	if ( dist_between_legs < leg_pair_separation_m )
	{
	  closest = it1;
	  closest_dist = (*it1)->dist_to_person_;
	  closest_dist_between_legs = dist_between_legs;
	}
      }
      // If we found a close, unlabeled leg, set it's label.
      if (closest != end) 
      {
	cout << "Replaced one leg with a distance of " << closest_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
	(*closest)->object_id = people_meas->object_id;
      }
      else 
      {
	cout << "Returned one matched leg only" << endl;
      }
      
      // Regardless of whether we found a second leg, return.
      return;
    }

    cout << "Looking for a pair of new legs" << endl;
    // If we didn't find any legs with this person's label, try to find two unlabeled legs that are close together and close to the tracker.
    it1 = saved_features_.begin();
    it2 = saved_features_.begin();
    closest = saved_features_.end();
    closest1 = saved_features_.end();
    closest2 = saved_features_.end();
    closest_dist = max_meas_jump_m;
    closest_pair_dist = 2*max_meas_jump_m;
    for (; it1 != end; ++it1) 
    {
      // Only look at trackers without ids and that are not too far away.
      if ((*it1)->object_id != "" || (*it1)->dist_to_person_ >= max_meas_jump_m )
	continue;

      // Keep the single closest leg around in case none of the pairs work out.
      if ( (*it1)->dist_to_person_ < closest_dist ) 
      {
	closest_dist = (*it1)->dist_to_person_;
	closest = it1;
      }

      // Find a second leg.
      it2 = it1;
      it2++;
      for (; it2 != end; ++it2) 
      {
	// Only look at trackers without ids and that are not too far away.
	if ((*it2)->object_id != "" || (*it2)->dist_to_person_ >= max_meas_jump_m ) 
	  continue;
   
	// Get the distance between the two legs
    try {
	tfl_.transformPoint((*it1)->id_, (*it2)->position_.stamp_, (*it2)->position_, fixed_frame, dest_loc);
    } catch(...) {
		ROS_WARN("TF exception getting distance between legs in spot 2.");
	}
	dist_between_legs = dest_loc.length();

	// Ensure that this pair of legs is the closest pair to the tracker, and that the distance between the legs isn't too large.
	if ( (*it1)->dist_to_person_+(*it2)->dist_to_person_ < closest_pair_dist && dist_between_legs < leg_pair_separation_m ) 
	{
	  closest_pair_dist = (*it1)->dist_to_person_+(*it2)->dist_to_person_;
	  closest1 = it1;
	  closest2 = it2;
	  closest_dist_between_legs = dist_between_legs;
	}
      }
    }
    // Found a pair of legs.
    if (closest1 != end && closest2 != end) 
    {
      (*closest1)->object_id = people_meas->object_id;
      (*closest2)->object_id = people_meas->object_id;
      cout << "Found a completely new pair with total distance " << closest_pair_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
      return;
    }

    cout << "Looking for just one leg" << endl;
    // No pair worked, try for just one leg.
    if (closest != end)
    {
      (*closest)->object_id = people_meas->object_id;
      cout << "Returned one new leg only" << endl;
      return;
    }

    cout << "Nothing matched" << endl;
  }





  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    ScanProcessor processor(*scan, mask_);

    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(5);

    CvMat* tmp_mat = cvCreateMat(1,feat_count_,CV_32FC1);

    // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);
    list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end())
    {
      if ((*sf_iter)->meas_time_ < purge)
      {
        delete (*sf_iter);
        saved_features_.erase(sf_iter++);
      }
      else
        ++sf_iter;
    }


    // System update of trackers, and copy updated ones in propagate list
    list<SavedFeature*> propagated;
    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++)
    {
      (*sf_iter)->propagate(scan->header.stamp);
      propagated.push_back(*sf_iter);
    }


    // Detection step: build up the set of "candidate" clusters
    list<SampleSet*> candidates;
    for (list<SampleSet*>::iterator i = processor.getClusters().begin();
         i != processor.getClusters().end();
         i++)
    {
      vector<float> f = calcLegFeatures(*i, *scan);

      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)(f[k]);

      if (forest.predict( tmp_mat ) > 0)
      {
        candidates.push_back(*i);
      }
    }


    // For each candidate, find the closest tracker (within threshold) and add to the match list
    // If no tracker is found, start a new one
    multiset<MatchedFeature> matches;
    for (list<SampleSet*>::iterator cf_iter = candidates.begin();
         cf_iter != candidates.end(); cf_iter++){
      Stamped<Point> loc((*cf_iter)->center(), scan->header.stamp, scan->header.frame_id);
		try {
		  tfl_.transformPoint(fixed_frame, loc, loc);
		} catch(...) {
    		ROS_WARN("TF exception spot 3.");
    	}

      list<SavedFeature*>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_m;
      
      for (list<SavedFeature*>::iterator pf_iter = propagated.begin();
           pf_iter != propagated.end();
           pf_iter++)
      {
        // find the closest distance between candidate and trackers
        float dist = loc.distance((*pf_iter)->position_);
        if ( dist < closest_dist )
        {
          closest = pf_iter;
          closest_dist = dist;
        }
      }
      // Nothing close to it, start a new track
      if (closest == propagated.end()) 
      {
        list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
      }
      // Add the candidate, the tracker and the distance to a match list
      else
        matches.insert(MatchedFeature(*cf_iter,*closest,closest_dist));
    }




    // loop through _sorted_ matches list
    // find the match with the shortest distance for each tracker
    while (matches.size() > 0)
    {
      multiset<MatchedFeature>::iterator matched_iter = matches.begin();
      bool found = false;
      list<SavedFeature*>::iterator pf_iter = propagated.begin();
      while (pf_iter != propagated.end())
      {
	// update the tracker with this candidate
        if (matched_iter->closest_ == *pf_iter)
        {
	  // Transform candidate to fixed frame
          Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
          try {
          tfl_.transformPoint(fixed_frame, loc, loc);
          } catch(...) {
          	ROS_WARN("TF exception spot 4.");
          }

	  // Update the tracker with the candidate location
          matched_iter->closest_->update(loc);
          
	  // remove this match and 
          matches.erase(matched_iter);
          propagated.erase(pf_iter++);
          found = true;
          break;
        }
	// still looking for the tracker to update
        else
        {
          pf_iter++;
        }
      }

      // didn't find tracker to update, because it was deleted above
      // try to assign the candidate to another tracker
      if (!found)
      {
        Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
        try {
        	tfl_.transformPoint(fixed_frame, loc, loc);
        } catch(...) {
        	ROS_WARN("TF exception spot 5.");
        }

        list<SavedFeature*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;
      
        for (list<SavedFeature*>::iterator remain_iter = propagated.begin();
             remain_iter != propagated.end();
             remain_iter++)
        {
          float dist = loc.distance((*remain_iter)->position_);
          if ( dist < closest_dist )
          {
            closest = remain_iter;
            closest_dist = dist;
          }
        }

	// no tracker is within a threshold of this candidate
	// so create a new tracker for this candidate
        if (closest == propagated.end())
          list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
        else
          matches.insert(MatchedFeature(matched_iter->candidate_,*closest,closest_dist));
	matches.erase(matched_iter);
      }
    }

    cvReleaseMat(&tmp_mat); tmp_mat = 0;

    vector<geometry_msgs::Point32> filter_visualize(saved_features_.size());
    vector<float> weights(saved_features_.size());
    sensor_msgs::ChannelFloat32 channel;
    int i = 0;

    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++,i++)
    {
      // reliability
      StatePosVel est;
      (*sf_iter)->filter_.getEstimate(est);
      double reliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));

      // publish filter result
      filter_visualize[i].x = est.pos_[0];
      filter_visualize[i].y = est.pos_[1];
      filter_visualize[i].z = est.pos_[2];
      weights[i] = *(float*)&(rgb[min(998, max(1, (int)trunc( reliability*999.0 )))]);

      people_msgs::PositionMeasurement pos;
      pos.header.stamp = (*sf_iter)->time_;
      pos.header.frame_id = fixed_frame;
      pos.name = "leg_detector";
      pos.object_id = (*sf_iter)->object_id;
      pos.pos.x = est.pos_[0];
      pos.pos.y = est.pos_[1];
      pos.pos.z = est.pos_[2];
      pos.covariance[0] = pow(0.3 / reliability,2.0);
      pos.covariance[1] = 0.0;
      pos.covariance[2] = 0.0;
      pos.covariance[3] = 0.0;
      pos.covariance[4] = pow(0.3 / reliability,2.0);
      pos.covariance[5] = 0.0;
      pos.covariance[6] = 0.0;
      pos.covariance[7] = 0.0;
      pos.covariance[8] = 10000.0;
      pos.initialization = 0;
      
      // If I've already seen this leg, publish its position.
      if ((*sf_iter)->object_id != "")
	tracker_measurements_pub_.publish(pos);        
    }

    // visualize all trackers
    channel.name = "rgb";
    channel.values = weights;
    sensor_msgs::PointCloud  people_cloud; 
    people_cloud.channels.push_back(channel);
    people_cloud.header.frame_id = fixed_frame;//scan_.header.frame_id;
    people_cloud.header.stamp = scan->header.stamp;
    people_cloud.points  = filter_visualize;
    leg_cloud_pub_.publish(people_cloud);

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv,"laser_processor");
  g_argc = argc;
  g_argv = argv;
  ros::NodeHandle nh;
  LegDetector ld(nh);
  ros::spin();
  
  return 0;
}

