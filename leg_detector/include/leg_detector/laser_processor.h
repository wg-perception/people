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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

//! A namespace containing the laser processor helper classes


#ifndef LASER_SCAN_LASERPROCESSOR_HH
#define LASER_SCAN_LASERPROCESSOR_HH

#include <unistd.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"

#include <list>
#include <set>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>

#include "tf/transform_datatypes.h"

namespace laser_processor
{
//! A struct representing a single sample from the laser.
class Sample
{
public:
  int   index;
  float range;
  float intensity;
  float x;
  float y;

  static Sample* Extract(int ind, const sensor_msgs::LaserScan& scan);

private:
  Sample() {};
};

//! The comparator allowing the creation of an ordered "SampleSet"
struct CompareSample
{
  CompareSample() {}

  inline bool operator()(const Sample* a, const Sample* b)
  {
    return (a->index <  b->index);
  }
};


//! An ordered set of Samples
class SampleSet : public std::set<Sample*, CompareSample>
{
public:
  SampleSet() {}

  ~SampleSet()
  {
    clear();
  }

  void clear();

  void appendToCloud(sensor_msgs::PointCloud& cloud, int r = 0, int g = 0, int b = 0);

  tf::Point center();
};

//! A mask for filtering out Samples based on range
class ScanMask
{
  SampleSet mask_;

  bool     filled;
  float    angle_min;
  float    angle_max;
  uint32_t size;

public:

  ScanMask() : filled(false), angle_min(0), angle_max(0), size(0) { }

  inline void clear()
  {
    mask_.clear();
    filled = false;
  }

  void addScan(sensor_msgs::LaserScan& scan);

  bool hasSample(Sample* s, float thresh);
};



class ScanProcessor
{
  std::list<SampleSet*> clusters_;
  sensor_msgs::LaserScan scan_;

public:

  std::list<SampleSet*>& getClusters()
  {
    return clusters_;
  }

  ScanProcessor(const sensor_msgs::LaserScan& scan, ScanMask& mask_, float mask_threshold = 0.03);

  ~ScanProcessor();

  void removeLessThan(uint32_t num);

  void splitConnected(float thresh);
};
};

#endif
