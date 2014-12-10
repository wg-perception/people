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


#include "people_tracking_filter/sysmodel_pos_vel.h"


using namespace std;
using namespace BFL;
using namespace tf;


static const unsigned int NUM_SYS_POS_VEL_COND_ARGS = 1;
static const unsigned int DIM_SYS_POS_VEL           = 6;


// Constructor
SysPdfPosVel::SysPdfPosVel(const StatePosVel& sigma)
  : ConditionalPdf<StatePosVel, StatePosVel>(DIM_SYS_POS_VEL, NUM_SYS_POS_VEL_COND_ARGS),
    noise_(StatePosVel(Vector3(0, 0, 0), Vector3(0, 0, 0)), sigma)
{}



// Destructor
SysPdfPosVel::~SysPdfPosVel()
{}



Probability
SysPdfPosVel::ProbabilityGet(const StatePosVel& state) const
{
  cerr << "SysPdfPosVel::ProbabilityGet Method not applicable" << endl;
  assert(0);
  return 0;
}


bool
SysPdfPosVel::SampleFrom(Sample<StatePosVel>& one_sample, int method, void *args) const
{
  StatePosVel& res = one_sample.ValueGet();

  // get conditional argument: state
  res = this->ConditionalArgumentGet(0);

  // apply system model
  res.pos_ += (res.vel_ * dt_);

  // add noise
  Sample<StatePosVel> noise_sample;
  noise_.SetDt(dt_);
  noise_.SampleFrom(noise_sample, method, args);
  res += noise_sample.ValueGet();

  return true;
}


StatePosVel
SysPdfPosVel::ExpectedValueGet() const
{
  cerr << "SysPdfPosVel::ExpectedValueGet Method not applicable" << endl;
  assert(0);
  return StatePosVel();

}

SymmetricMatrix
SysPdfPosVel::CovarianceGet() const
{
  cerr << "SysPdfPosVel::CovarianceGet Method not applicable" << endl;
  SymmetricMatrix Covar(DIM_SYS_POS_VEL);
  assert(0);
  return Covar;
}

