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

#ifndef __TRACKER_KALMAN__
#define __TRACKER_KALMAN__

#include "tracker.h"

// bayesian filtering
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>


#include "state_pos_vel.h"

// TF
#include <tf/tf.h>

// log files
#include <fstream>

namespace estimation
{

class TrackerKalman: public Tracker
{
public:
  /// constructor
  TrackerKalman(const std::string& name, const BFL::StatePosVel& sysnoise);

  /// destructor
  virtual ~TrackerKalman();

  /// initialize tracker
  virtual void initialize(const BFL::StatePosVel& mu, const BFL::StatePosVel& sigma, const double time);

  /// return if tracker was initialized
  virtual bool isInitialized() const
  {
    return tracker_initialized_;
  };

  /// return measure for tracker quality: 0=bad 1=good
  virtual double getQuality() const
  {
    return quality_;
  };

  /// return the lifetime of the tracker
  virtual double getLifetime() const;

  /// return the time of the tracker
  virtual double getTime() const;

  /// update tracker
  virtual bool updatePrediction(const double time);
  virtual bool updateCorrection(const tf::Vector3& meas,
                                const MatrixWrapper::SymmetricMatrix& cov);

  /// get filter posterior
  virtual void getEstimate(BFL::StatePosVel& est) const;
  virtual void getEstimate(people_msgs::PositionMeasurement& est) const;


private:
  // pdf / model / filter
  BFL::Gaussian                                           prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  BFL::LinearAnalyticConditionalGaussian*                 sys_pdf_;
  BFL::LinearAnalyticSystemModelGaussianUncertainty*      sys_model_;
  BFL::LinearAnalyticConditionalGaussian*                 meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model_;
  MatrixWrapper::Matrix                                   sys_matrix_;
  MatrixWrapper::SymmetricMatrix                          sys_sigma_;

  double calculateQuality();

  // vars
  bool tracker_initialized_;
  double init_time_, filter_time_, quality_;


}; // class

}; // namespace

#endif
