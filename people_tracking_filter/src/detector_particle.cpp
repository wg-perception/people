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

#include "people_tracking_filter/detector_particle.h"
#include "people_tracking_filter/uniform_vector.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;
using namespace geometry_msgs;




namespace estimation
{
// constructor
DetectorParticle::DetectorParticle(unsigned int num_particles):
  prior_(num_particles),
  filter_(NULL),
  sys_model_(tf::Vector3(0.1, 0.1, 0.1)),
  meas_model_(tf::Vector3(0.1, 0.1, 0.1)),
  detector_initialized_(false),
  num_particles_(num_particles)
{}



// destructor
DetectorParticle::~DetectorParticle()
{
  if (filter_) delete filter_;
}


// initialize prior density of filter
void DetectorParticle::initialize(const tf::Vector3& mu, const tf::Vector3& size, const double time)
{
  cout << "Initializing detector with " << num_particles_ << " particles, with uniform size "
       << size << " around " << mu << endl;

  UniformVector uniform_vector(mu, size);
  vector<Sample<tf::Vector3> > prior_samples(num_particles_);
  uniform_vector.SampleFrom(prior_samples, num_particles_, CHOLESKY, NULL);
  prior_.ListOfSamplesSet(prior_samples);
  filter_ = new BootstrapFilter<tf::Vector3, tf::Vector3>(&prior_, &prior_, 0, num_particles_ / 4.0);

  // detector initialized
  detector_initialized_ = true;
  quality_ = 1;
  filter_time_ = time;
}




// update filter prediction
bool DetectorParticle::updatePrediction(const double dt)
{
  // set de in sys model
  sys_model_.SetDt(dt);

  // update filter
  bool res = filter_->Update(&sys_model_);
  if (!res) quality_ = 0;

  return res;
}



// update filter correction
bool DetectorParticle::updateCorrection(const tf::Vector3&  meas, const MatrixWrapper::SymmetricMatrix& cov, const double time)
{
  assert(cov.columns() == 3);

  // set filter time
  filter_time_ = time;

  // set covariance
  ((MeasPdfVector*)(meas_model_.MeasurementPdfGet()))->CovarianceSet(cov);

  // update filter
  bool res = filter_->Update(&meas_model_, meas);
  if (!res) quality_ = 0;

  return res;
}


// get evenly spaced particle cloud
void DetectorParticle::getParticleCloud(const tf::Vector3& step, double threshold, sensor_msgs::PointCloud& cloud) const
{
  ((MCPdfVector*)(filter_->PostGet()))->getParticleCloud(step, threshold, cloud);
}


// get most recent filter posterior
void DetectorParticle::getEstimate(tf::Vector3& est) const
{
  est = ((MCPdfVector*)(filter_->PostGet()))->ExpectedValueGet();
}


void DetectorParticle::getEstimate(people_msgs::PositionMeasurement& est) const
{
  tf::Vector3 tmp = filter_->PostGet()->ExpectedValueGet();

  est.pos.x = tmp[0];
  est.pos.y = tmp[1];
  est.pos.z = tmp[2];

  est.header.stamp.fromSec(filter_time_);
  est.header.frame_id = "base_link";
}





/// Get histogram from certain area
Matrix DetectorParticle::getHistogram(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const
{
  return ((MCPdfVector*)(filter_->PostGet()))->getHistogram(min, max, step);
}


}; // namespace



