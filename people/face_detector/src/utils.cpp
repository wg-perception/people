/**********************************************************************
*
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

#include "utils.h"

using namespace std;

// Computes the median of the non-zero values in array arr between rows r1-r2 (inclusive) 
// and columns c1-c2 (inclusive).
// This is often used as a robust replacement for a mean.
// Currently only supports 1-channel images, and cannot use ROI or COI settings.
// Default return value is 0.0.
double cvMedianNonZeroElIn2DArr(CvArr *arr, int r1, int r2, int c1, int c2) {

  // Convert OpenCV array types CvMat, IplImage, CvMatND to the CvMat format.
  int type = cvGetElemType(arr);
  CvMat *mat = cvCreateMatHeader(cvGetDimSize(arr,0), cvGetDimSize(arr,1), type);  
  cvGetMat(arr, mat);

  r1 = MAX(0,r1);
  r2 = MIN(cvGetDimSize(arr,0)-1,r2);
  c1 = MAX(0,c1);
  c2 = MIN(cvGetDimSize(arr,1)-1,c2);

  int r, c;
  
  // Create a vector of non-zero elements in the array boundaries, 
  // sort them, and then take the middle value. This is the median.
  if (type == CV_8UC1) {    
    std::vector<uint8_t> elems;
    uint8_t* ptr;
    for (r = r1; r<=r2; r++) {
      ptr = (uint8_t*)(mat->data.ptr + r*mat->step) + c1;
      for (c = c1; c<=c2; c++) {
	if ((*ptr)!=0.0) {
	  elems.push_back(*ptr);
	}
	ptr++;
      }
    }
    if (elems.size() == 0) {
      return 0;
    }
    sort(elems.begin(), elems.end()); // Sort in ascending order.
    return (double)elems[floor(elems.size()/2)];
  }
  else if (type== CV_8SC1) {
    std::vector<int8_t> elems;
    int8_t* ptr;
    for (r = r1; r<=r2; r++) {
      ptr = (int8_t*)(mat->data.ptr + r*mat->step) + c1;
      for (c = c1; c<=c2; c++) {
	if ((*ptr)!=0.0) {
	  elems.push_back(*ptr);
	}
	ptr++;
      }
    }
    if (elems.size() == 0) {
      return 0;
    }
    sort(elems.begin(), elems.end()); // Sort in ascending order.
    return (double)elems[floor(elems.size()/2)];
  }
  else if (type== CV_16UC1) {
    std::vector<uint16_t> elems;
    uint16_t* ptr;
    for (r = r1; r<=r2; r++) {
      ptr = (uint16_t*)(mat->data.ptr + r*mat->step) + c1;
      for (c = c1; c<=c2; c++) {
	if ((*ptr)!=0.0) {
	  elems.push_back(*ptr);
	}
	ptr++;
      }
    }
    if (elems.size() == 0) {
      return 0;
    }
    sort(elems.begin(), elems.end()); // Sort in ascending order.
    return (double)elems[floor(elems.size()/2)];
  }
  else if (type== CV_16SC1) {
    std::vector<int16_t> elems;
    int16_t* ptr;
    for (r = r1; r<=r2; r++) {
      ptr = (int16_t*)(mat->data.ptr + r*mat->step) + c1;
      for (c = c1; c<=c2; c++) {
	if ((*ptr)!=0.0) {
	  elems.push_back(*ptr);
	}
	ptr++;
      }
    }
    if (elems.size() == 0) {
      return 0;
    }
    sort(elems.begin(), elems.end()); // Sort in ascending order.
    return (double)elems[floor(elems.size()/2)];
  }
  else if (type == CV_32SC1 ) {
    std::vector<int32_t> elems;
    int32_t* ptr;
    for (r = r1; r<=r2; r++) {
      ptr = (int32_t*)(mat->data.ptr + r*mat->step) + c1;
      for (c = c1; c<=c2; c++) {
	if ((*ptr)!=0.0) {
	  elems.push_back(*ptr);
	}
	ptr++;
      }
    }
    if (elems.size() == 0) {
      return 0;
    }
    sort(elems.begin(), elems.end()); // Sort in ascending order.
    return (double)elems[floor(elems.size()/2)];
  }
  else if (type == CV_32FC1) {
    std::vector<float> elems;
    float* ptr;
    for (r = r1; r<=r2; r++) {
      ptr = (float*)(mat->data.ptr + r*mat->step) + c1;
      for (c = c1; c<=c2; c++) {
	if ((*ptr)!=0.0) {
	  elems.push_back(*ptr);
	}
	ptr++;
      }
    }
    if (elems.size() == 0) {
      return 0;
    }
    sort(elems.begin(), elems.end()); // Sort in ascending order.
    return (double)elems[floor(elems.size()/2)];
  }
  else if (type == CV_64FC1 ){
    std::vector<double> elems;
    double* ptr;
    for (r = r1; r<=r2; r++) {
      ptr = (double*)(mat->data.ptr + r*mat->step) + c1;
      for (c = c1; c<=c2; c++) {
	if ((*ptr)!=0.0) {
	  elems.push_back(*ptr);
	}
	ptr++;
      }
    }
    if (elems.size() == 0) {
      return 0;
    }
    sort(elems.begin(), elems.end()); // Sort in ascending order.
    return (double)elems[floor(elems.size()/2)];
  }
  else {
    printf("Array type not supported by cvMedianNonZeroElIn2DArr.\n");
  }

  return 0;
}

