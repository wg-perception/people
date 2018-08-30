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

#ifndef UNIFORM_VECTOR_H
#define UNIFORM_VECTOR_H

#include <bfl/pdf/pdf.h>
#include <tf/tf.h>



namespace BFL
{
/// Class representing uniform vector
class UniformVector: public Pdf<tf::Vector3>
{
private:
  tf::Vector3 mu_, size_;
  double probability_;

public:
  /// Constructor
  UniformVector(const tf::Vector3& mu, const tf::Vector3& size);

  /// Destructor
  virtual ~UniformVector();

  /// output stream for UniformVector
  friend std::ostream& operator<< (std::ostream& os, const UniformVector& g);

  // Redefinition of pure virtuals
  virtual UniformVector* Clone() const;

  // Redefinition of pure virtuals
  virtual Probability ProbabilityGet(const tf::Vector3& input) const;
  bool SampleFrom(vector<Sample<tf::Vector3> >& list_samples, const int num_samples, int method = DEFAULT, void * args = NULL) const;
  virtual bool SampleFrom(Sample<tf::Vector3>& one_sample, int method = DEFAULT, void * args = NULL) const;

  virtual tf::Vector3 ExpectedValueGet() const;
  virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;

};

} // end namespace
#endif
