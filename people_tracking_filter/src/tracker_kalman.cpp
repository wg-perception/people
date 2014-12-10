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

#include "people_tracking_filter/tracker_kalman.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;


const static double damping_velocity = 0.9;


namespace estimation
{
// constructor
TrackerKalman::TrackerKalman(const string& name, const StatePosVel& sysnoise):
  Tracker(name),
  filter_(NULL),
  sys_pdf_(NULL),
  sys_model_(NULL),
  meas_pdf_(NULL),
  meas_model_(NULL),
  sys_matrix_(6, 6),
  tracker_initialized_(false)
{
  // create sys model
  sys_matrix_ = 0;
  for (unsigned int i = 1; i <= 3; i++)
  {
    sys_matrix_(i, i) = 1;
    sys_matrix_(i + 3, i + 3) = damping_velocity;
  }
  ColumnVector sys_mu(6);
  sys_mu = 0;
  sys_sigma_ = SymmetricMatrix(6);
  sys_sigma_ = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    sys_sigma_(i + 1, i + 1) = pow(sysnoise.pos_[i], 2);
    sys_sigma_(i + 4, i + 4) = pow(sysnoise.vel_[i], 2);
  }
  Gaussian sys_noise(sys_mu, sys_sigma_);
  sys_pdf_   = new LinearAnalyticConditionalGaussian(sys_matrix_, sys_noise);
  sys_model_ = new LinearAnalyticSystemModelGaussianUncertainty(sys_pdf_);


  // create meas model
  Matrix meas_matrix(3, 6);
  meas_matrix = 0;
  for (unsigned int i = 1; i <= 3; i++)
    meas_matrix(i, i) = 1;

  ColumnVector meas_mu(3);
  meas_mu = 0;
  SymmetricMatrix meas_sigma(3);
  meas_sigma = 0;
  for (unsigned int i = 0; i < 3; i++)
    meas_sigma(i + 1, i + 1) = 0;
  Gaussian meas_noise(meas_mu, meas_sigma);
  meas_pdf_   = new LinearAnalyticConditionalGaussian(meas_matrix, meas_noise);
  meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);
};



// destructor
TrackerKalman::~TrackerKalman()
{
  if (filter_)      delete filter_;
  if (sys_pdf_)     delete sys_pdf_;
  if (sys_model_)   delete sys_model_;
  if (meas_pdf_)    delete meas_pdf_;
  if (meas_model_)  delete meas_model_;
};



// initialize prior density of filter
void TrackerKalman::initialize(const StatePosVel& mu, const StatePosVel& sigma, const double time)
{
  ColumnVector mu_vec(6);
  SymmetricMatrix sigma_vec(6);
  sigma_vec = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    mu_vec(i + 1) = mu.pos_[i];
    mu_vec(i + 4) = mu.vel_[i];
    sigma_vec(i + 1, i + 1) = pow(sigma.pos_[i], 2);
    sigma_vec(i + 4, i + 4) = pow(sigma.vel_[i], 2);
  }
  prior_ = Gaussian(mu_vec, sigma_vec);
  filter_ = new ExtendedKalmanFilter(&prior_);

  // tracker initialized
  tracker_initialized_ = true;
  quality_ = 1;
  filter_time_ = time;
  init_time_ = time;
}




// update filter prediction
bool TrackerKalman::updatePrediction(const double time)
{
  bool res = true;
  if (time > filter_time_)
  {
    // set dt in sys model
    for (unsigned int i = 1; i <= 3; i++)
      sys_matrix_(i, i + 3) = time - filter_time_;
    sys_pdf_->MatrixSet(0, sys_matrix_);

    // scale system noise for dt
    sys_pdf_->AdditiveNoiseSigmaSet(sys_sigma_ * pow(time - filter_time_, 2));
    filter_time_ = time;

    // update filter
    res = filter_->Update(sys_model_);
    if (!res) quality_ = 0;
    else quality_ = calculateQuality();
  }
  return res;
};



// update filter correction
bool TrackerKalman::updateCorrection(const tf::Vector3&  meas, const MatrixWrapper::SymmetricMatrix& cov)
{
  assert(cov.columns() == 3);

  // copy measurement
  ColumnVector meas_vec(3);
  for (unsigned int i = 0; i < 3; i++)
    meas_vec(i + 1) = meas[i];

  // set covariance
  ((LinearAnalyticConditionalGaussian*)(meas_model_->MeasurementPdfGet()))->AdditiveNoiseSigmaSet(cov);

  // update filter
  bool res = filter_->Update(meas_model_, meas_vec);
  if (!res) quality_ = 0;
  else quality_ = calculateQuality();

  return res;
};


void TrackerKalman::getEstimate(StatePosVel& est) const
{
  ColumnVector tmp = filter_->PostGet()->ExpectedValueGet();
  for (unsigned int i = 0; i < 3; i++)
  {
    est.pos_[i] = tmp(i + 1);
    est.vel_[i] = tmp(i + 4);
  }
};


void TrackerKalman::getEstimate(people_msgs::PositionMeasurement& est) const
{
  ColumnVector tmp = filter_->PostGet()->ExpectedValueGet();

  est.pos.x = tmp(1);
  est.pos.y = tmp(2);
  est.pos.z = tmp(3);

  est.header.stamp.fromSec(filter_time_);
  est.object_id = getName();
}




double TrackerKalman::calculateQuality()
{
  double sigma_max = 0;
  SymmetricMatrix cov = filter_->PostGet()->CovarianceGet();
  for (unsigned int i = 1; i <= 2; i++)
    sigma_max = max(sigma_max, sqrt(cov(i, i)));

  return 1.0 - min(1.0, sigma_max / 1.5);
}


double TrackerKalman::getLifetime() const
{
  if (tracker_initialized_)
    return filter_time_ - init_time_;
  else
    return 0;
}

double TrackerKalman::getTime() const
{
  if (tracker_initialized_)
    return filter_time_;
  else
    return 0;
}

}; // namespace


