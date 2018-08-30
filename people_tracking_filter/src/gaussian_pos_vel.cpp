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


#include "people_tracking_filter/gaussian_pos_vel.h"
#include <bfl/wrappers/rng/rng.h>
#include <cmath>
#include <cassert>

using namespace tf;

namespace BFL
{
GaussianPosVel::GaussianPosVel(const StatePosVel& mu, const StatePosVel& sigma)
  : Pdf<StatePosVel> (1),
    mu_(mu),
    sigma_(sigma),
    gauss_pos_(mu.pos_, sigma.pos_),
    gauss_vel_(mu.vel_, sigma.vel_)
{}


GaussianPosVel::~GaussianPosVel() {}

GaussianPosVel* GaussianPosVel::Clone() const
{
  return new GaussianPosVel(mu_, sigma_);
}

std::ostream& operator<< (std::ostream& os, const GaussianPosVel& g)
{
  os << "\nMu pos :\n"    << g.ExpectedValueGet().pos_ << endl
     << "\nMu vel :\n"    << g.ExpectedValueGet().vel_ << endl
     << "\nSigma:\n" << g.CovarianceGet() << endl;
  return os;
}


Probability GaussianPosVel::ProbabilityGet(const StatePosVel& input) const
{
  return gauss_pos_.ProbabilityGet(input.pos_) * gauss_vel_.ProbabilityGet(input.vel_);
}


bool
GaussianPosVel::SampleFrom(vector<Sample<StatePosVel> >& list_samples, const int num_samples, int method, void * args) const
{
  list_samples.resize(num_samples);
  vector<Sample<StatePosVel> >::iterator sample_it = list_samples.begin();
  for (sample_it = list_samples.begin(); sample_it != list_samples.end(); sample_it++)
    SampleFrom(*sample_it, method, args);

  return true;
}


bool
GaussianPosVel::SampleFrom(Sample<StatePosVel>& one_sample, int method, void * args) const
{
  one_sample.ValueSet(StatePosVel(Vector3(rnorm(mu_.pos_[0], sigma_.pos_[0]*dt_),
                                          rnorm(mu_.pos_[1], sigma_.pos_[1]*dt_),
                                          rnorm(mu_.pos_[2], sigma_.pos_[2]*dt_)),
                                  Vector3(rnorm(mu_.vel_[0], sigma_.vel_[0]*dt_),
                                          rnorm(mu_.vel_[1], sigma_.vel_[1]*dt_),
                                          rnorm(mu_.vel_[2], sigma_.vel_[2]*dt_))));
  return true;
}


StatePosVel
GaussianPosVel::ExpectedValueGet() const
{
  return mu_;
}

SymmetricMatrix
GaussianPosVel::CovarianceGet() const
{
  SymmetricMatrix sigma(6);
  sigma = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    sigma(i + 1, i + 1) = pow(sigma_.pos_[i], 2);
    sigma(i + 4, i + 4) = pow(sigma_.vel_[i], 2);
  }
  return sigma;
}

} // End namespace BFL
