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

#ifndef __TRACKER__
#define __TRACKER__

#include "state_pos_vel.h"
#include <people_msgs/PositionMeasurement.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <string>


namespace estimation
{

class Tracker
{
public:
  /// constructor
  Tracker(const std::string& name): name_(name) {};

  /// destructor
  virtual ~Tracker() {};

  /// return the name of the tracker
  const std::string& getName() const
  {
    return name_;
  };

  /// initialize tracker
  virtual void initialize(const BFL::StatePosVel& mu, const BFL::StatePosVel& sigma, const double time) = 0;

  /// return if tracker was initialized
  virtual bool isInitialized() const = 0;

  /// return measure for tracker quality: 0=bad 1=good
  virtual double getQuality() const = 0;

  /// return the lifetime of the tracker
  virtual double getLifetime() const = 0;

  /// return the time of the tracker
  virtual double getTime() const = 0;

  /// update tracker
  virtual bool updatePrediction(const double time) = 0;
  virtual bool updateCorrection(const tf::Vector3& meas,
                                const MatrixWrapper::SymmetricMatrix& cov) = 0;

  /// get filter posterior
  virtual void getEstimate(BFL::StatePosVel& est) const = 0;
  virtual void getEstimate(people_msgs::PositionMeasurement& est) const = 0;

private:
  std::string name_;

}; // class

}; // namespace

#endif
