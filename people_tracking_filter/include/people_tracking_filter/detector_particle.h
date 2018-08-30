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

#ifndef __DETECTOR_PARTICLE__
#define __DETECTOR_PARTICLE__

#include "tracker.h"

// bayesian filtering
#include <bfl/filter/bootstrapfilter.h>
#include "mcpdf_vector.h"
#include "measmodel_vector.h"
#include "sysmodel_vector.h"

// TF
#include <tf/tf.h>

// msgs
#include <sensor_msgs/PointCloud.h>

// log files
#include <fstream>

namespace estimation
{

class DetectorParticle
{
public:
  /// constructor
  DetectorParticle(unsigned int num_particles);

  /// destructor
  ~DetectorParticle();

  /// initialize detector
  void initialize(const tf::Vector3& mu, const tf::Vector3& size, const double time);

  /// return if detector was initialized
  bool isInitialized() const
  {
    return detector_initialized_;
  };

  /// return measure for detector quality: 0=bad 1=good
  double getQuality() const
  {
    return quality_;
  };

  /// update detector
  bool updatePrediction(const double dt);
  bool updateCorrection(const tf::Vector3& meas,
                        const MatrixWrapper::SymmetricMatrix& cov,
                        const double time);

  /// get filter posterior
  void getEstimate(tf::Vector3& est) const;
  void getEstimate(people_msgs::PositionMeasurement& est) const;

  // get evenly spaced particle cloud
  void getParticleCloud(const tf::Vector3& step, double threshold, sensor_msgs::PointCloud& cloud) const;

  /// Get histogram from certain area
  MatrixWrapper::Matrix getHistogram(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const;

private:
  // pdf / model / filter
  BFL::MCPdfVector                                          prior_;
  BFL::BootstrapFilter<tf::Vector3, tf::Vector3>* filter_;
  BFL::SysModelVector                                       sys_model_;
  BFL::MeasModelVector                                      meas_model_;

  // vars
  bool detector_initialized_;
  double filter_time_, quality_;
  unsigned int num_particles_;


}; // class

}; // namespace

#endif
