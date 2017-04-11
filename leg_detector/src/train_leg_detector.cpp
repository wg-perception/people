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

#include "leg_detector/laser_processor.h"
#include "leg_detector/calc_leg_features.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/query.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/ml.h"

#include "people_msgs/PositionMeasurement.h"
#include "sensor_msgs/LaserScan.h"

#include <boost/foreach.hpp>  // for rosbag iterator

using namespace laser_processor;
using namespace ros;

enum LoadType {LOADING_NONE, LOADING_POS, LOADING_NEG, LOADING_TEST};

class TrainLegDetector
{
public:
  ScanMask mask_;
  int mask_count_;

  std::vector< std::vector<float> > pos_data_;
  std::vector< std::vector<float> > neg_data_;
  std::vector< std::vector<float> > test_data_;

  CvRTrees forest;

  float connected_thresh_;

  int feat_count_;

  TrainLegDetector() : mask_count_(0), connected_thresh_(0.06), feat_count_(0)
  {
  }

  void loadData(LoadType load, char* file)
  {
    if (load != LOADING_NONE)
    {
      switch (load)
      {
      case LOADING_POS:
        printf("Loading positive training data from file: %s\n", file);
        break;
      case LOADING_NEG:
        printf("Loading negative training data from file: %s\n", file);
        break;
      case LOADING_TEST:
        printf("Loading test data from file: %s\n", file);
        break;
      default:
        break;
      }

      rosbag::Bag bag;
      try
      {
        bag.open(file, rosbag::bagmode::Read);
      }
      catch (rosbag::BagException)
      {
        ROS_FATAL_STREAM("Cannot open " << file);
        exit(-1);
      }

      mask_.clear();
      mask_count_ = 0;

      rosbag::View view(bag, rosbag::TypeQuery("sensor_msgs/LaserScan"));
      BOOST_FOREACH(rosbag::MessageInstance const m, view)
      {
        sensor_msgs::LaserScan::Ptr msg = m.instantiate<sensor_msgs::LaserScan>();
        switch (load)
        {
        case LOADING_POS:
          loadScan(msg.get(), pos_data_);
          break;
        case LOADING_NEG:
          mask_count_ = 1000; // effectively disable masking
          loadScan(msg.get(), neg_data_);
          break;
        case LOADING_TEST:
          loadScan(msg.get(), test_data_);
          break;
        default:
          break;
        }
      }
    }
  }

  void loadScan(sensor_msgs::LaserScan* scan, std::vector< std::vector<float> >& data)
  {
    if (mask_count_++ < 20)
    {
      mask_.addScan(*scan);
    }
    else
    {
      ScanProcessor processor(*scan, mask_);
      processor.splitConnected(connected_thresh_);
      processor.removeLessThan(5);

      for (std::list<SampleSet*>::iterator i = processor.getClusters().begin();
           i != processor.getClusters().end();
           i++)
      {
        data.push_back(calcLegFeatures(*i, *scan));
        feat_count_ = data[0].size();
      }
    }
  }

  void train()
  {
    int sample_size = pos_data_.size() + neg_data_.size();

    CvMat* cv_data = cvCreateMat(sample_size, feat_count_, CV_32FC1);
    CvMat* cv_resp = cvCreateMat(sample_size, 1, CV_32S);

    // Put positive data in opencv format.
    int j = 0;
    for (std::vector< std::vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step * j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];

      cv_resp->data.i[j] = 1;
      j++;
    }

    // Put negative data in opencv format.
    for (std::vector< std::vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step * j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];

      cv_resp->data.i[j] = -1;
      j++;
    }

    CvMat* var_type = cvCreateMat(1, feat_count_ + 1, CV_8U);
    cvSet(var_type, cvScalarAll(CV_VAR_ORDERED));
    cvSetReal1D(var_type, feat_count_, CV_VAR_CATEGORICAL);

    float priors[] = {1.0, 1.0};

    CvRTParams fparam(8, 20, 0, false, 10, priors, false, 5, 50, 0.001f, CV_TERMCRIT_ITER);
    fparam.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1);

    forest.train(cv_data, CV_ROW_SAMPLE, cv_resp, 0, 0, var_type, 0,
                 fparam);


    cvReleaseMat(&cv_data);
    cvReleaseMat(&cv_resp);
    cvReleaseMat(&var_type);
  }

  void test()
  {
    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

    int pos_right = 0;
    int pos_total = 0;
    for (std::vector< std::vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) > 0)
        pos_right++;
      pos_total++;
    }

    int neg_right = 0;
    int neg_total = 0;
    for (std::vector< std::vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) < 0)
        neg_right++;
      neg_total++;
    }

    int test_right = 0;
    int test_total = 0;
    for (std::vector< std::vector<float> >::iterator i = test_data_.begin();
         i != test_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) > 0)
        test_right++;
      test_total++;
    }

    if (!pos_data_.empty())
      printf(" Pos train set: %d/%d %g\n", pos_right, pos_total, (float)(pos_right) / pos_total);
    if (!neg_data_.empty())
      printf(" Neg train set: %d/%d %g\n", neg_right, neg_total, (float)(neg_right) / neg_total);
    if (!test_data_.empty())
      printf(" Test set:      %d/%d %g\n", test_right, test_total, (float)(test_right) / test_total);

    cvReleaseMat(&tmp_mat);
  }

  bool load(char* file)
  {
    forest.load(file);
    return (forest.get_active_var_mask()->cols != 0);
  }

  void save(char* file)
  {
    forest.save(file);
  }
};

int main(int argc, char **argv)
{
  TrainLegDetector tld;

  LoadType loading = LOADING_NONE;

  char save_file[100];
  char load_file[100];
  save_file[0] = load_file[0] = 0;

  printf("Loading data...\n");
  for (int i = 1; i < argc; i++)
  {
    if (!strcmp(argv[i], "--pos"))
      loading = LOADING_POS;
    else if (!strcmp(argv[i], "--neg"))
      loading = LOADING_NEG;
    else if (!strcmp(argv[i], "--test"))
      loading = LOADING_TEST;
    else if (!strcmp(argv[i], "--load"))
    {
      if (++i < argc)
        strncpy(load_file, argv[i], 100);
      continue;
    }
    else if (!strcmp(argv[i], "--save"))
    {
      if (++i < argc)
        strncpy(save_file, argv[i], 100);
      continue;
    }
    else
      tld.loadData(loading, argv[i]);
  }

  if (strlen(load_file) > 0)
  {
    printf("Loading classifier from: %s\n", load_file);
    if (!tld.load(load_file))
    {
      printf("**Unable to load classifier**");
      return -1;
    }
  }
  else
  {
    printf("Training classifier...\n");
    tld.train();
  }

  printf("Evlauating classifier...\n");
  tld.test();

  if (strlen(save_file) > 0)
  {
    printf("Saving classifier as: %s\n", save_file);
    tld.save(save_file);
  }

  return 0;
}
