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

#include <people_tracking_filter/sysmodel_vector.h>

static const unsigned int NUM_SYS_VECTOR_COND_ARGS  = 1;
static const unsigned int DIM_SYS_VECTOR            = 3;

namespace BFL
{
// Constructor
SysPdfVector::SysPdfVector(const tf::Vector3& sigma)
  : ConditionalPdf<tf::Vector3, tf::Vector3>(DIM_SYS_VECTOR, NUM_SYS_VECTOR_COND_ARGS),
    noise_(tf::Vector3(0, 0, 0), sigma)
{}

// Destructor
SysPdfVector::~SysPdfVector()
{}

Probability
SysPdfVector::ProbabilityGet(const tf::Vector3& state) const
{
  std::cerr << "SysPdfVector::ProbabilityGet Method not applicable" << std::endl;
  assert(0);
  return 0;
}

bool
SysPdfVector::SampleFrom(Sample<tf::Vector3>& one_sample, int method, void *args) const
{
  tf::Vector3& res = one_sample.ValueGet();

  // get conditional argument: state
  res = this->ConditionalArgumentGet(0);

  // add noise
  Sample<tf::Vector3> noise_sample;
  noise_.SampleFrom(noise_sample, method, args);
  res += noise_sample.ValueGet();

  return true;
}

tf::Vector3
SysPdfVector::ExpectedValueGet() const
{
  std::cerr << "SysPdfVector::ExpectedValueGet Method not applicable" << std::endl;
  assert(0);
  return tf::Vector3();
}

SymmetricMatrix
SysPdfVector::CovarianceGet() const
{
  std::cerr << "SysPdfVector::CovarianceGet Method not applicable" << std::endl;
  SymmetricMatrix Covar(DIM_SYS_VECTOR);
  assert(0);
  return Covar;
}
}  // namespace BFL
