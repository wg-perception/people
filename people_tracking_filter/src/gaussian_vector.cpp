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

#include "people_tracking_filter/gaussian_vector.h"
#include <wrappers/rng/rng.h>
#include <cmath>
#include <cassert>

using namespace tf;

namespace BFL
{
GaussianVector::GaussianVector(const Vector3& mu, const Vector3& sigma)
  : Pdf<Vector3> (1),
    mu_(mu),
    sigma_(sigma),
    sigma_changed_(true)
{
  for (unsigned int i = 0; i < 3; i++)
    assert(sigma[i] > 0);
}


GaussianVector::~GaussianVector() {}


std::ostream& operator<< (std::ostream& os, const GaussianVector& g)
{
  os << "Mu   :\n" << g.ExpectedValueGet() << endl
     << "Sigma:\n" << g.CovarianceGet() << endl;
  return os;
}

void GaussianVector::sigmaSet(const Vector3& sigma)
{
  sigma_ = sigma;
  sigma_changed_ = true;
}

Probability GaussianVector::ProbabilityGet(const Vector3& input) const
{
  if (sigma_changed_)
  {
    sigma_changed_ = false;
    // 2 * sigma^2
    for (unsigned int i = 0; i < 3; i++)
      sigma_sq_[i] = 2 * sigma_[i] * sigma_[i];
    // sqrt
    sqrt_ = 1 / sqrt(M_PI * M_PI * M_PI * sigma_sq_[0] * sigma_sq_[1] * sigma_sq_[2]);
  }

  Vector3 diff = input - mu_;
  return sqrt_ * exp(- (diff[0] * diff[0] / sigma_sq_[0])
                     - (diff[1] * diff[1] / sigma_sq_[1])
                     - (diff[2] * diff[2] / sigma_sq_[2]));
}


bool
GaussianVector::SampleFrom(vector<Sample<Vector3> >& list_samples, const int num_samples, int method, void * args) const
{
  list_samples.resize(num_samples);
  vector<Sample<Vector3> >::iterator sample_it = list_samples.begin();
  for (sample_it = list_samples.begin(); sample_it != list_samples.end(); sample_it++)
    SampleFrom(*sample_it, method, args);

  return true;
}


bool
GaussianVector::SampleFrom(Sample<Vector3>& one_sample, int method, void * args) const
{
  one_sample.ValueSet(Vector3(rnorm(mu_[0], sigma_[0]),
                              rnorm(mu_[1], sigma_[1]),
                              rnorm(mu_[2], sigma_[2])));
  return true;
}


Vector3
GaussianVector::ExpectedValueGet() const
{
  return mu_;
}

SymmetricMatrix
GaussianVector::CovarianceGet() const
{
  SymmetricMatrix sigma(3);
  sigma = 0;
  for (unsigned int i = 0; i < 3; i++)
    sigma(i + 1, i + 1) = pow(sigma_[i], 2);
  return sigma;
}

GaussianVector*
GaussianVector::Clone() const
{
  return new GaussianVector(mu_, sigma_);
}

} // End namespace BFL
