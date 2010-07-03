/*********************************************************************
 * Faces-specific computer vision algorithms.
 *
 **********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Caroline Pantofaru
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

#include "face_detector/faces.h"
#include <cfloat>

#define __FACES_DEBUG__ 0
#define __FACES_DISPLAY__ 0


Faces::Faces():
  list_(NULL),
  cv_image_gray_(0),
  disparity_image_(0),
  cam_model_(NULL) {
}

Faces::~Faces() {
  // Kill all the threads.
  threads_.interrupt_all();
  threads_.join_all();
  delete face_go_mutex_;
  face_go_mutex_ = NULL;

  for (int i=list_.size(); i>0; i--) {
    list_.pop_back();
  }

  if (cv_image_gray_) {cvReleaseImage(&cv_image_gray_); cv_image_gray_ = 0; }
  disparity_image_ = 0;
  cam_model_ = 0;

  if (storage_) {cvReleaseMemStorage(&storage_); storage_ = 0;}
  if (cascade_) {cvReleaseHaarClassifierCascade(&cascade_); cascade_ = 0;}

}



/********
 * Detect all faces in an image.
 * Input:
 * image - The image in which to detect faces.
 * haar_classifier_filename - Path to the xml file containing the trained haar classifier cascade.
 * threshold - Detection threshold. Currently unused.
 * cam_model - The camera model.
 * disparity_image - Image of disparities (from stereo). To avoid using depth information, set this to NULL.
 * do_draw - If true, draw a box on `image' around each face.
 * Output:
 * A vector of CvRects containing the bounding boxes around found faces.
 *********/ 



void Faces::initFaceDetection(uint num_cascades, string haar_classifier_filename) {
  images_ready_ = 0;

  face_go_mutex_ = new boost::mutex();
  cascade_ = (CvHaarClassifierCascade*)cvLoad(haar_classifier_filename.c_str());
  
  if (!cascade_) {
    std::cerr << "Cascade file " << haar_classifier_filename << " doesn't exist.\n" << std::endl;
    return;
  }
  storage_ = cvCreateMemStorage(0);
  threads_.create_thread(boost::bind(&Faces::faceDetectionThread,this,0));
}

/////

vector<Box2D3D> Faces::detectAllFaces(IplImage *image, double threshold, IplImage *disparity_image, image_geometry::StereoCameraModel *cam_model) {

  faces_.clear();

  CvSize im_size = cvGetSize(image);

  // Convert the image to grayscale, if necessary.
  if (cv_image_gray_==NULL) {
    cv_image_gray_ = cvCreateImage(im_size, 8, 1);
  }
  if (image->nChannels == 1) {
    cvCopy(image, cv_image_gray_, NULL);
  }
  else if (image->nChannels == 3) {
    cvCvtColor(image, cv_image_gray_, CV_BGR2GRAY);
  } 
  else {
    std::cerr << "Unknown image type"<<std::endl;
    return faces_;
  }
  disparity_image_ = disparity_image;
  cam_model_ = cam_model;
  
  // Tell the face detection threads that they can run once.
  num_threads_to_wait_for_ = threads_.size();
  boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
  images_ready_++;
  fgmlock.unlock();

  face_detection_ready_cond_.notify_all();

  boost::mutex::scoped_lock fdmlock(face_done_mutex_);
  while (num_threads_to_wait_for_ > 0) {
    face_detection_done_cond_.wait(fdmlock);
  }  

  return faces_;
}

/////

void Faces::faceDetectionThread(uint i) {

  while (1) {
    boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
    boost::mutex::scoped_lock tlock(t_mutex_, boost::defer_lock);
    while (1) {
      tlock.lock();
      if (images_ready_) {
	--images_ready_;
	tlock.unlock();
	break;
      }
      tlock.unlock();
      face_detection_ready_cond_.wait(fgmlock);
    }

    // Find the faces using OpenCV's haar cascade object detector.
    cv::Point3d p3_1(0,0,MAX_Z_M);
    cv::Point3d p3_2(FACE_SIZE_MIN_M,0,MAX_Z_M);
    cv::Point2d p2_1, p2_2;
    (cam_model_->left()).project3dToPixel(p3_1,p2_1);
    (cam_model_->left()).project3dToPixel(p3_2,p2_2);
    int this_min_face_size = (int)(floor(fabs(p2_2.x-p2_1.x)));

    CvSeq *face_seq = cvHaarDetectObjects(cv_image_gray_, cascade_, storage_, 1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(this_min_face_size,this_min_face_size));

    // Filter the faces using depth information, if available. Currently checks that the actual face size is within the given limits.
    CvScalar color = cvScalar(0,255,0);
    Box2D3D one_face;
    double avg_disp = 0.0;
    CvMat *uvd = cvCreateMat(1,3,CV_32FC1);
    CvMat *xyz = cvCreateMat(1,3,CV_32FC1);
    // For each face...
    for (int iface = 0; iface < face_seq->total; iface++) {

      one_face.status = "good";

      one_face.box2d = *(CvRect *)cvGetSeqElem(face_seq,iface);
      one_face.id = i; // The cascade that computed this face.

      if (disparity_image_) {
    
	// Get the median disparity in the middle half of the bounding box.
	avg_disp = cvMedianNonZeroElIn2DArr(disparity_image_,
					    floor(one_face.box2d.y+0.25*one_face.box2d.height),floor(one_face.box2d.y+0.75*one_face.box2d.height),
					    floor(one_face.box2d.x+0.25*one_face.box2d.width), floor(one_face.box2d.x+0.75*one_face.box2d.width)); 

	one_face.center2d = cvScalar(one_face.box2d.x+one_face.box2d.width/2.0,
				     one_face.box2d.y+one_face.box2d.height/2.0,
				     avg_disp);
	one_face.radius2d = one_face.box2d.width/2.0;

	if (avg_disp > 0) {
	  p2_1.x = 0; p2_1.y = 0; p2_2.x = one_face.box2d.width; p2_2.y = 0;
	  cam_model_->projectDisparityTo3d(p2_1,avg_disp,p3_1);
	  cam_model_->projectDisparityTo3d(p2_2,avg_disp,p3_2);
	  one_face.radius3d = fabs(p3_2.x-p3_1.x)/2.0;
	  p2_1.x = one_face.center2d.val[0];
	  p2_1.y = one_face.center2d.val[1];
	  cam_model_->projectDisparityTo3d(p2_1, avg_disp, p3_1);
	  one_face.center3d = cvScalar(p3_1.x,p3_1.y,p3_1.z);
	  if (one_face.center3d.val[3] > MAX_Z_M || 2.0*one_face.radius3d < FACE_SIZE_MIN_M || 2.0*one_face.radius3d > FACE_SIZE_MAX_M) {
	    one_face.status = "bad";
	  }
	}
	else {
	  one_face.radius3d = 0.0;     
	  one_face.center3d = cvScalar(0.0,0.0,0.0);
	  one_face.status = "unknown";
	}
      }

      // Add faces to the output vector.
      // lock faces
      boost::mutex::scoped_lock lock(face_mutex_);
      faces_.push_back(one_face);
      lock.unlock();
    }

    cvClearSeq(face_seq);

    cvReleaseMat(&uvd); uvd = 0;
    cvReleaseMat(&xyz); xyz = 0;

    boost::mutex::scoped_lock fdmlock(face_done_mutex_);
    num_threads_to_wait_for_--;
    fdmlock.unlock();
    face_detection_done_cond_.notify_one();
  }
}

