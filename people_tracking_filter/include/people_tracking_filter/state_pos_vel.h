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


#ifndef STATE_POS_VEL_H
#define STATE_POS_VEL_H

#include <tf/tf.h>

namespace BFL
{
/// Class representing state with pos and vel
class StatePosVel
{
public:
  tf::Vector3 pos_, vel_;

  /// Constructor
  StatePosVel(const tf::Vector3& pos = tf::Vector3(0, 0, 0),
              const tf::Vector3& vel = tf::Vector3(0, 0, 0)):  pos_(pos), vel_(vel) {};

  /// Destructor
  ~StatePosVel() {};

  /// operator +=
  StatePosVel& operator += (const StatePosVel& s)
  {
    this->pos_ += s.pos_;
    this->vel_ += s.vel_;
    return *this;
  }

  /// operator +
  StatePosVel operator + (const StatePosVel& s)
  {
    StatePosVel res;

    res.pos_ = this->pos_ + s.pos_;
    res.vel_ = this->vel_ + s.vel_;
    return res;
  }

  /// output stream for StatePosVel
  friend std::ostream& operator<< (std::ostream& os, const StatePosVel& s)
  {
    os << "(" << s.pos_[0] << ", " << s.pos_[1] << ", "  << s.pos_[2] << ")--("
       << "(" << s.vel_[0] << ", " << s.vel_[1] << ", "  << s.vel_[2] << ") ";
    return os;
  };




};
} // end namespace
#endif
