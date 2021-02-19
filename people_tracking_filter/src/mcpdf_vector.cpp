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

#include <people_tracking_filter/mcpdf_vector.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <assert.h>
#include <vector>
#include <std_msgs/Float64.h>
#include <people_tracking_filter/rgb.h>
#include <algorithm>

namespace BFL
{
static const unsigned int NUM_CONDARG   = 1;

MCPdfVector::MCPdfVector(unsigned int num_samples)
  : MCPdf<tf::Vector3> (num_samples, NUM_CONDARG)
{}

MCPdfVector::~MCPdfVector() {}

WeightedSample<tf::Vector3>
MCPdfVector::SampleGet(unsigned int particle) const
{
  assert(static_cast<int>(particle) >= 0 && particle < _listOfSamples.size());
  return _listOfSamples[particle];
}

tf::Vector3 MCPdfVector::ExpectedValueGet() const
{
  tf::Vector3 pos(0, 0, 0);
  double current_weight;
  std::vector<WeightedSample<tf::Vector3> >::const_iterator it_los;
  for (it_los = _listOfSamples.begin() ; it_los != _listOfSamples.end() ; it_los++)
  {
    current_weight = it_los->WeightGet();
    pos += (it_los->ValueGet() * current_weight);
  }

  return tf::Vector3(pos);
}

/// Get evenly distributed particle cloud
void MCPdfVector::getParticleCloud(const tf::Vector3& step, double threshold, sensor_msgs::PointCloud& cloud) const
{
  unsigned int num_samples = _listOfSamples.size();
  assert(num_samples > 0);
  tf::Vector3 m = _listOfSamples[0].ValueGet();
  tf::Vector3 M = _listOfSamples[0].ValueGet();

  // calculate min and max
  for (unsigned int s = 0; s < num_samples; s++)
  {
    tf::Vector3 v = _listOfSamples[s].ValueGet();
    for (unsigned int i = 0; i < 3; i++)
    {
      if (v[i] < m[i]) m[i] = v[i];
      if (v[i] > M[i]) M[i] = v[i];
    }
  }

  // get point cloud from histogram
  Matrix hist = getHistogram(m, M, step);
  unsigned int row = hist.rows();
  unsigned int col = hist.columns();
  unsigned int total = 0;
  unsigned int t = 0;
  for (unsigned int r = 1; r <= row; r++)
    for (unsigned int c = 1; c <= col; c++)
      if (hist(r, c) > threshold) total++;
  std::cout << "size total " << total << std::endl;

  std::vector<geometry_msgs::Point32> points(total);
  std::vector<float> weights(total);
  sensor_msgs::ChannelFloat32 channel;
  for (unsigned int r = 1; r <= row; r++)
    for (unsigned int c = 1; c <= col; c++)
      if (hist(r, c) > threshold)
      {
        for (unsigned int i = 0; i < 3; i++)
          points[t].x = m[0] + (step[0] * r);
        points[t].y = m[1] + (step[1] * c);
        points[t].z = m[2];
        weights[t] = rgb[999 - static_cast<int>(trunc(std::max(0.0, std::min(999.0, hist(r, c) * 2 * total * total))))];
        t++;
      }
  std::cout << "points size " << points.size() << std::endl;
  cloud.header.frame_id = "base_link";
  cloud.points  = points;
  channel.name = "rgb";
  channel.values = weights;
  cloud.channels.push_back(channel);
}

/// Get histogram from pos
MatrixWrapper::Matrix MCPdfVector::getHistogram(const tf::Vector3& m,
                                                const tf::Vector3& M,
                                                const tf::Vector3& step) const
{
  unsigned int num_samples = _listOfSamples.size();
  unsigned int rows = round((M[0] - m[0]) / step[0]);
  unsigned int cols = round((M[1] - m[1]) / step[1]);
  Matrix hist(rows, cols);
  hist = 0;

  // calculate histogram
  for (unsigned int i = 0; i < num_samples; i++)
  {
    tf::Vector3 rel(_listOfSamples[i].ValueGet() - m);
    unsigned int r = round(rel[0] / step[0]);
    unsigned int c = round(rel[1] / step[1]);
    if (r >= 1 && c >= 1 && r <= rows && c <= cols)
      hist(r, c) += _listOfSamples[i].WeightGet();
  }

  return hist;
}

unsigned int
MCPdfVector::numParticlesGet() const
{
  return _listOfSamples.size();
}
}  // namespace BFL
