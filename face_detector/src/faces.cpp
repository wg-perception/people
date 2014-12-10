/*********************************************************************
 * Faces-specific computer vision algorithms.
 *
 **********************************************************************
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

/* Author: Caroline Pantofaru */

#include "face_detector/faces.h"
#include <cfloat>
#include <algorithm>

namespace people
{

Faces::Faces():
  list_(NULL),
  cam_model_(NULL)
{
}

Faces::~Faces()
{
  // Kill all the threads.
  threads_.interrupt_all();
  threads_.join_all();
  delete face_go_mutex_;
  face_go_mutex_ = NULL;

  for (int i = list_.size(); i > 0; i--)
  {
    list_.pop_back();
  }

  cam_model_ = 0;

}


/* Note: The multi-threading in this file is left over from a previous incarnation that allowed multiple
 * cascades to be run at once. It hasn't been removed in case we want to return to that model, and since
 * there isn't much overhead. Right now, however, only one classifier is being run per instantiation
 * of the face_detector node.
 */


void Faces::initFaceDetectionDisparity(uint num_cascades, string haar_classifier_filename, double face_size_min_m, double face_size_max_m, double max_face_z_m, double face_sep_dist_m)
{
  images_ready_ = 0;

  face_size_min_m_ = face_size_min_m;
  face_size_max_m_ = face_size_max_m;
  max_face_z_m_ = max_face_z_m;
  face_sep_dist_m_ = face_sep_dist_m;

  face_go_mutex_ = new boost::mutex();
  bool cascade_ok = cascade_.load(haar_classifier_filename);

  if (!cascade_ok)
  {
    ROS_ERROR_STREAM("Cascade file " << haar_classifier_filename << " doesn't exist.");
    return;
  }
  threads_.create_thread(boost::bind(&Faces::faceDetectionThreadDisparity, this, 0));
}

/////

vector<Box2D3D> Faces::detectAllFacesDisparity(const cv::Mat &image, double threshold, const cv::Mat &disparity_image, image_geometry::StereoCameraModel *cam_model)
{

  faces_.clear();

  // Convert the image to grayscale, if necessary.

  if (image.channels() == 1)
  {
    cv_image_gray_.create(image.size(), CV_8UC1);
    image.copyTo(cv_image_gray_);
  }
  else if (image.channels() == 3)
  {
    cv_image_gray_.create(image.size(), CV_8UC1);
    cv::cvtColor(image, cv_image_gray_, CV_BGR2GRAY);
  }
  else
  {
    std::cerr << "Unknown image type" << std::endl;
    return faces_;
  }

  disparity_image_ = &disparity_image;
  cam_model_ = cam_model;

  // Tell the face detection threads that they can run once.
  num_threads_to_wait_for_ = threads_.size();
  boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
  images_ready_++;
  fgmlock.unlock();

  face_detection_ready_cond_.notify_all();

  boost::mutex::scoped_lock fdmlock(face_done_mutex_);
  while (num_threads_to_wait_for_ > 0)
  {
    face_detection_done_cond_.wait(fdmlock);
  }

  return faces_;
}

/////

void Faces::faceDetectionThreadDisparity(uint i)
{

  while (1)
  {
    boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
    boost::mutex::scoped_lock tlock(t_mutex_, boost::defer_lock);
    while (1)
    {
      tlock.lock();
      if (images_ready_)
      {
        --images_ready_;
        tlock.unlock();
        break;
      }
      tlock.unlock();
      face_detection_ready_cond_.wait(fgmlock);
    }

    // Find the faces using OpenCV's haar cascade object detector.
    cv::Point3d p3_1(0, 0, max_face_z_m_);
    cv::Point3d p3_2(face_size_min_m_, 0, max_face_z_m_);
    cv::Point2d p2_1 = (cam_model_->left()).project3dToPixel(p3_1);
    cv::Point2d p2_2 = (cam_model_->left()).project3dToPixel(p3_2);
    int this_min_face_size = (int)(floor(fabs(p2_2.x - p2_1.x)));

    std::vector<cv::Rect> faces_vec;
    cascade_.detectMultiScale(cv_image_gray_, faces_vec,  1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cv::Size(this_min_face_size, this_min_face_size));

    // Filter the faces using depth information, if available. Currently checks that the actual face size is within the given limits.
    cv::Scalar color(0, 255, 0);
    Box2D3D one_face;
    double avg_disp = 0.0;
    cv::Mat uvd(1, 3, CV_32FC1);
    cv::Mat xyz(1, 3, CV_32FC1);
    // For each face...
    for (uint iface = 0; iface < faces_vec.size(); iface++)  //face_seq->total; iface++) {
    {

      one_face.status = "good";

      one_face.box2d = faces_vec[iface];
      one_face.id = i; // The cascade that computed this face.

      // Get the median disparity in the middle half of the bounding box.
      cv::Mat disp_roi_shallow(*disparity_image_, cv::Rect(floor(one_face.box2d.x + 0.25 * one_face.box2d.width),
                               floor(one_face.box2d.y + 0.25 * one_face.box2d.height),
                               floor(one_face.box2d.x + 0.75 * one_face.box2d.width) - floor(one_face.box2d.x + 0.25 * one_face.box2d.width) + 1,
                               floor(one_face.box2d.y + 0.75 * one_face.box2d.height) - floor(one_face.box2d.y + 0.25 * one_face.box2d.height) + 1));
      cv::Mat disp_roi = disp_roi_shallow.clone();
      cv::Mat tmat = disp_roi.reshape(1, disp_roi.rows * disp_roi.cols);
      cv::Mat tmat_sorted;
      cv::sort(tmat, tmat_sorted, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
      avg_disp = tmat_sorted.at<float>(floor(cv::countNonZero(tmat_sorted >= 0.0) / 2.0)); // Get the middle valid disparity (-1 disparities are invalid)

      // Fill in the rest of the face data structure.
      one_face.center2d = cv::Point2d(one_face.box2d.x + one_face.box2d.width / 2.0,
                                      one_face.box2d.y + one_face.box2d.height / 2.0);
      one_face.width2d = one_face.box2d.width;

      // If the median disparity was valid and the face is a reasonable size, the face status is "good".
      // If the median disparity was valid but the face isn't a reasonable size, the face status is "bad".
      // Otherwise, the face status is "unknown".
      if (avg_disp > 0)
      {
        cam_model_->projectDisparityTo3d(cv::Point2d(0.0, 0.0), avg_disp, p3_1);
        cam_model_->projectDisparityTo3d(cv::Point2d(one_face.box2d.width, 0.0), avg_disp, p3_2);
        one_face.width3d = fabs(p3_2.x - p3_1.x);
        cam_model_->projectDisparityTo3d(one_face.center2d, avg_disp, one_face.center3d);
        if (one_face.center3d.z > max_face_z_m_ || one_face.width3d < face_size_min_m_ || one_face.width3d > face_size_max_m_)
        {
          one_face.status = "bad";
        }
      }
      else
      {
        one_face.width3d = 0.0;
        one_face.center3d = cv::Point3d(0.0, 0.0, 0.0);
        one_face.status = "unknown";
      }

      // Add faces to the output vector.
      // lock faces
      boost::mutex::scoped_lock lock(face_mutex_);
      faces_.push_back(one_face);
      lock.unlock();
    }

    boost::mutex::scoped_lock fdmlock(face_done_mutex_);
    num_threads_to_wait_for_--;
    fdmlock.unlock();
    face_detection_done_cond_.notify_one();
  }
}

/////

void Faces::initFaceDetectionDepth(uint num_cascades, string haar_classifier_filename, double face_size_min_m, double face_size_max_m, double max_face_z_m, double face_sep_dist_m)
{

  images_ready_ = 0;

  face_size_min_m_ = face_size_min_m;
  face_size_max_m_ = face_size_max_m;
  max_face_z_m_ = max_face_z_m;
  face_sep_dist_m_ = face_sep_dist_m;

  face_go_mutex_ = new boost::mutex();
  bool cascade_ok = cascade_.load(haar_classifier_filename);

  if (!cascade_ok)
  {
    ROS_ERROR_STREAM("Cascade file " << haar_classifier_filename << " doesn't exist.");
    return;
  }
  threads_.create_thread(boost::bind(&Faces::faceDetectionThreadDepth, this, 0));
}


/////

vector<Box2D3D> Faces::detectAllFacesDepth(const cv::Mat &image, double threshold, const cv::Mat &dimage, image_geometry::StereoCameraModel *cam_model)
{

  faces_.clear();

  // Convert the image to grayscale, if necessary.

  if (image.channels() == 1)
  {
    cv_image_gray_.create(image.size(), CV_8UC1);
    image.copyTo(cv_image_gray_);
  }
  else if (image.channels() == 3)
  {
    cv_image_gray_.create(image.size(), CV_8UC1);
    cv::cvtColor(image, cv_image_gray_, CV_BGR2GRAY);
  }
  else
  {
    std::cerr << "Unknown image type" << std::endl;
    return faces_;
  }

  depth_image_ = &dimage;
  cam_model_ = cam_model;

  // Tell the face detection threads that they can run once.
  num_threads_to_wait_for_ = threads_.size();
  boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
  images_ready_++;
  fgmlock.unlock();

  face_detection_ready_cond_.notify_all();

  boost::mutex::scoped_lock fdmlock(face_done_mutex_);
  while (num_threads_to_wait_for_ > 0)
  {
    face_detection_done_cond_.wait(fdmlock);
  }

  return faces_;
}

/////

void Faces::faceDetectionThreadDepth(uint i)
{

  while (1)
  {
    boost::mutex::scoped_lock fgmlock(*(face_go_mutex_));
    boost::mutex::scoped_lock tlock(t_mutex_, boost::defer_lock);
    while (1)
    {
      tlock.lock();
      if (images_ready_)
      {
        --images_ready_;
        tlock.unlock();
        break;
      }
      tlock.unlock();
      face_detection_ready_cond_.wait(fgmlock);
    }

    // Find the faces using OpenCV's haar cascade object detector.
    cv::Point3d p3_1(0, 0, max_face_z_m_);
    cv::Point3d p3_2(face_size_min_m_, 0, max_face_z_m_);
    cv::Point2d p2_1 = (cam_model_->left()).project3dToPixel(p3_1);
    cv::Point2d p2_2 = (cam_model_->left()).project3dToPixel(p3_2);
    int this_min_face_size = (int)(floor(fabs(p2_2.x - p2_1.x)));

    std::vector<cv::Rect> faces_vec;
    cascade_.detectMultiScale(cv_image_gray_, faces_vec,  1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cv::Size(this_min_face_size, this_min_face_size));

    // Filter the faces using depth information, if available. Currently checks that the actual face size is within the given limits.
    cv::Scalar color(0, 255, 0);
    Box2D3D one_face;
    double avg_d = 0.0;
    cv::Mat uvd(1, 3, CV_32FC1);
    cv::Mat xyz(1, 3, CV_32FC1);
    // For each face...
    for (uint iface = 0; iface < faces_vec.size(); iface++)
    {

      one_face.status = "good";

      one_face.box2d = faces_vec[iface];
      one_face.id = i; // The cascade that computed this face.

      // Get the median depth in the middle half of the bounding box.
      cv::Mat depth_roi_shallow(*depth_image_, cv::Rect(floor(one_face.box2d.x + 0.25 * one_face.box2d.width),
                                floor(one_face.box2d.y + 0.25 * one_face.box2d.height),
                                floor(one_face.box2d.x + 0.75 * one_face.box2d.width) - floor(one_face.box2d.x + 0.25 * one_face.box2d.width) + 1,
                                floor(one_face.box2d.y + 0.75 * one_face.box2d.height) - floor(one_face.box2d.y + 0.25 * one_face.box2d.height) + 1));

      // Fill in the rest of the face data structure.
      one_face.center2d = cv::Point2d(one_face.box2d.x + one_face.box2d.width / 2.0,
                                      one_face.box2d.y + one_face.box2d.height / 2.0);
      one_face.width2d = one_face.box2d.width;


      // If the median depth was valid and the face is a reasonable size, the face status is "good".
      // If the median depth was valid but the face isn't a reasonable size, the face status is "bad".
      // Otherwise, the face status is "unknown".

      std::vector<float> depths;
      // Copy the depths removing the nans.
      for (int i = 0; i < depth_roi_shallow.rows; i++)
      {
        const float* dptr = depth_roi_shallow.ptr<float>(i);
        for (int j = 0; j < depth_roi_shallow.cols; j++)
        {
          if (dptr[j] == dptr[j])
          {
            depths.push_back(dptr[j]);
          }
        }
      }

      std::vector<float>::iterator dbegin = depths.begin();
      std::vector<float>::iterator dend = depths.end();

      if (depths.size() > 0)
      {
        std::sort(dbegin, dend);
        avg_d = depths[floor(depths.size() / 2.0)];

        one_face.width3d = fabs((cam_model_->left()).getDeltaX(one_face.box2d.width, avg_d));
        one_face.center3d = (cam_model_->left()).projectPixelTo3dRay(one_face.center2d);

        one_face.center3d = (avg_d / one_face.center3d.z) * one_face.center3d;
        if (one_face.center3d.z > max_face_z_m_ || one_face.width3d < face_size_min_m_ || one_face.width3d > face_size_max_m_)
        {
          one_face.status = "bad";
        }
      }
      else
      {
        one_face.width3d = 0.0;
        one_face.center3d = cv::Point3d(0.0, 0.0, 0.0);
        one_face.status = "unknown";
      }

      // Add faces to the output vector.
      // lock faces
      boost::mutex::scoped_lock lock(face_mutex_);
      faces_.push_back(one_face);
      lock.unlock();
    }

    boost::mutex::scoped_lock fdmlock(face_done_mutex_);
    num_threads_to_wait_for_--;
    fdmlock.unlock();
    face_detection_done_cond_.notify_one();
  }

}

};
