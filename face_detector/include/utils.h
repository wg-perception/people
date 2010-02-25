/**********************************************************************
*
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc
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


#ifndef PEOPLE_UTILS_H
#define PEOPLE_UTILS_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>

using namespace std;

/** 
    A structure for holding information about boxes in 2d and 3d.
*/
struct Box2D3D {
  CvScalar center2d;
  CvScalar center3d;
  double radius2d;
  double radius3d;
  CvRect box2d;
  string status;
  int id;
};

// Computes the median of the non-zero values in array arr between rows r1-r2 (inclusive) 
// and columns c1-c2 (inclusive).
// This is often used as a robust replacement for a mean.
// Currently only supports 1-channel images, and cannot use ROI or COI settings.
// Default return value is 0.0.
double cvMedianNonZeroElIn2DArr(CvArr *arr, int r1, int r2, int c1, int c2);

#endif
