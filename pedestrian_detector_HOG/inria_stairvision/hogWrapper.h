//Author: Ian Goodfellow <ia3n@cs.stanford.edu>
//File: hogWrapper.h
//Description: stairvision wrapper around the OpenCV hog descriptor class


#pragma once


#include "svlVision.h"
#include "cvaux.hpp"

class HogWrapper :  public svlFeatureExtractor
{
 public:

  HogWrapper();

  HogWrapper(unsigned validChannel);

  HogWrapper(const HogWrapper & o);

  const HogWrapper & operator=(const HogWrapper & o);

  void extract(const vector<CvPoint> &locations,
		       const vector<svlDataFrame*> &frames,
		       vector< vector<double> > &output,
		       bool sparse,
	       int outputOffset = 0) const;

  bool writeOut(ofstream &ofs);

  bool load(XMLNode &root);

  svlFeatureExtractor* getPrunedExtractor
     (const vector<bool> &featureUsedBits) const;

  svlFeatureExtractor * clone() const;

  string summary() const;

  void setSize(CvSize size);

  unsigned numFeatures() const;

  ~HogWrapper();

 protected:
  
  svlSmartPointer<cv::HOGDescriptor> _hog;
  unsigned _validChannel;
};

SVL_AUTOREGISTER_H ( HogWrapper, svlFeatureExtractor );
