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

#include "people_tracking_filter/uniform_vector.h"
#include <wrappers/rng/rng.h>
#include <cmath>
#include <cassert>

using namespace tf;

namespace BFL
{
UniformVector::UniformVector(const Vector3& mu, const Vector3& size)
  : Pdf<Vector3> (1),
    mu_(mu),
    size_(size)
{
  for (unsigned int i = 0; i < 3; i++)
    assert(size_[i] > 0);

  probability_ = 1 / (size_[0] * 2 * size_[1] * 2 * size_[2] * 2);
}


UniformVector::~UniformVector() {}

UniformVector* UniformVector::Clone() const
{
  return new UniformVector(mu_, size_);
}

std::ostream& operator<< (std::ostream& os, const UniformVector& g)
{
  os << "Mu   :\n" << g.ExpectedValueGet() << endl
     << "Size :\n" << g.CovarianceGet() << endl;
  return os;
}



Probability UniformVector::ProbabilityGet(const Vector3& input) const
{
  for (unsigned int i = 0; i < 3; i++)
  {
    if (input[i] < (mu_[0] - (size_[0]))) return 0;
    if (input[i] > (mu_[0] + (size_[0]))) return 0;
  }
  return probability_;
}


bool
UniformVector::SampleFrom(vector<Sample<Vector3> >& list_samples, const int num_samples, int method, void * args) const
{
  list_samples.resize(num_samples);
  vector<Sample<Vector3> >::iterator sample_it = list_samples.begin();
  for (sample_it = list_samples.begin(); sample_it != list_samples.end(); sample_it++)
    SampleFrom(*sample_it, method, args);

  return true;
}


bool
UniformVector::SampleFrom(Sample<Vector3>& one_sample, int method, void * args) const
{
  one_sample.ValueSet(Vector3(((runif() - 0.5) * 2 * size_[0]) + mu_[0],
                              ((runif() - 0.5) * 2 * size_[1]) + mu_[1],
                              ((runif() - 0.5) * 2 * size_[2]) + mu_[2]));
  return true;
}


Vector3
UniformVector::ExpectedValueGet() const
{
  return mu_;
}

SymmetricMatrix
UniformVector::CovarianceGet() const
{
  SymmetricMatrix sigma(3);
  sigma = 0;
  for (unsigned int i = 0; i < 3; i++)
    sigma(i + 1, i + 1) = pow(size_[i], 2);
  return sigma;
}

} // End namespace BFL
