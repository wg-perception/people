/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2009, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    classifyImages.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**  Application for classifying images.
**
*****************************************************************************/

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlML.h"
#include "svlVision.h"
#include "hogWrapper.h"

using namespace std;

#define WINDOW_NAME "classifyImages"

// main -----------------------------------------------------------------------

void usage()
{
	//assert(false);
	cerr << SVL_USAGE_HEADER << endl;
	cerr << "USAGE: ./classifyImages [OPTIONS] <extractor> <model> (<image>* | <imageSequence>)" << endl;
	cerr << "OPTIONS:" << endl
	     << "  -b                :: only search at base scale" << endl
	     << "  -n <object name>  :: name of object" << endl
	     << "  -o <filename>     :: output to file" << endl
	     << "  -t <n>            :: threshold for outputting a detection" << endl
	     << "                       (note: svlSlidingWindow's threshold still determines" << endl
	     << "                        the min. probability for a detection to be found)" << endl
	     << "  -r <x> <y> <w> <h> :: scan subregion of image" << endl
	     << "  -R <filename>     :: only scan rectangles in file (<x> <y> <w> <h>)" <<endl
	     << "  -maxImages <n>    :: maximum number of images to analyze" <<endl
	     << "  -x                :: display image and matches" << endl
	     << "  -g <filename>     :: groundtruth file (to analyze on the spot)" << endl
	     << "  -pr <filename>    :: output a pr curve if groundtruth present (from summary scores)" << endl
	     << "  -s <filename>     :: output an evaluations summary if groundtruth present" << endl
	     << "  -cache <folder>   :: caches the results of the sliding window detection prior to non-maxima suppression" << endl
	     << SVL_STANDARD_OPTIONS_USAGE 
	     << endl;
}

int main(int argc, char *argv[])
{		
  captureOpenCVerrors();
  
  const unsigned MAX_RESULTS_TO_SHOW = 16;
  const unsigned MAX_WIDTH = 1024;

  // read commandline parameters  
  bool bBaseScaleOnly = false;
  const char *objectName = "[unknown]";
  const char *outputFilename = NULL;
  CvRect subRegion = cvRect(0, 0, 0, 0);
  char *regionsFile = NULL;
  bool bDisplayImage = false;
  bool bUseNonmax = false;
  int maxImages = INT_MAX;
  const char *groundTruth = NULL;
  const char *prFilename = NULL;
  const char *summaryFilename = NULL;
  float outputThreshold = 0.0;
  const char *cacheFolder = NULL;
  
  SVL_BEGIN_CMDLINE_PROCESSING(argc, argv)
    SVL_CMDLINE_BOOL_OPTION("-b", bBaseScaleOnly)
    SVL_CMDLINE_STR_OPTION("-n", objectName)
    SVL_CMDLINE_STR_OPTION("-o", outputFilename)
    SVL_CMDLINE_OPTION_BEGIN("-r", ptr)
      subRegion.x = atoi(ptr[0]);
      subRegion.y = atoi(ptr[1]);
      subRegion.width = atoi(ptr[2]);
      subRegion.height = atoi(ptr[3]);
    SVL_CMDLINE_OPTION_END(4)
    SVL_CMDLINE_STR_OPTION("-R", regionsFile)
    SVL_CMDLINE_INT_OPTION("-maxImages", maxImages)
    SVL_CMDLINE_BOOL_OPTION("-x", bDisplayImage)
    SVL_CMDLINE_BOOL_OPTION("-nm", bUseNonmax)      
    SVL_CMDLINE_STR_OPTION("-g", groundTruth)
    SVL_CMDLINE_STR_OPTION("-pr", prFilename)
    SVL_CMDLINE_STR_OPTION("-s", summaryFilename)
    SVL_CMDLINE_STR_OPTION("-cache", cacheFolder)
    SVL_CMDLINE_REAL_OPTION("-t", outputThreshold)
  SVL_END_CMDLINE_PROCESSING(usage());
    
  if (SVL_CMDLINE_ARGC < 3) {
    usage();
    return -1;
  }

  svlCodeProfiler::tic(svlCodeProfiler::getHandle("main"));
	
  const char *extractorFilename = *SVL_CMDLINE_ARGV++; SVL_CMDLINE_ARGC--;
  const char *classifierFilename = *SVL_CMDLINE_ARGV++; SVL_CMDLINE_ARGC--;

  int classifyHandle = svlCodeProfiler::getHandle("classification");

  svlImageLoader loader;
  loader.readCommandLine(SVL_CMDLINE_ARGV, SVL_CMDLINE_ARGC);

  svlSlidingWindowDetector detector(objectName);
  bool success = detector.load(extractorFilename, classifierFilename);
  SVL_ASSERT_MSG(success, "detector failed to load extractor or classifier");
  
  svlObjectDetectionAnalyzer analyzer(objectName);
  if (groundTruth) {
    analyzer.loadGt(groundTruth);
    analyzer.beginAccumulatingSummaryScores();
  }

  // run classifier on all supplied images
  unsigned numImages = loader.numFrames();
  if (maxImages < (int)numImages) numImages =  maxImages;

  vector<IplImage *> annotatedImages;
  annotatedImages.resize(((numImages < MAX_RESULTS_TO_SHOW) ?
			  numImages : MAX_RESULTS_TO_SHOW), NULL);
  if (sqrt((double)annotatedImages.size()) * loader.getResizeWidth() > MAX_WIDTH) {
    annotatedImages.resize(1);
  }
  IplImage *resultsImage = NULL;
  
  if (bDisplayImage) {
    cvNamedWindow(WINDOW_NAME, 0);
  }

  if (cacheFolder) {
    svlCreateDirectory(cacheFolder);
  }

  ofstream ofs;
  if (outputFilename) {
    ofs.open(outputFilename);
    assert(!ofs.fail());
    ofs << "<Object2dSequence>" << endl;
  }

  map<double, vector<CvPoint> > locations_map; // if cacheFolder

  SVL_LOG(SVL_LOG_MESSAGE, "Running classifier on " << numImages << " images/sequences...");

  double threshold= detector.getThreshold();
 
  for (unsigned index = 0; index < numImages; index++) 
    {
      SVL_LOG(SVL_LOG_VERBOSE, "Processing " << loader.getFrameBasename(index) << "...");

      vector<IplImage *> images;
      bool success = loader.getFrame(index, images);
      if (!success) continue;

      // classify image
      svlObject2dFrame objects;
      map<double, svlObject2dFrame> objects_map; // if cache folder, will use this first
 
      svlCodeProfiler::tic(classifyHandle);
      if (regionsFile != NULL) {
          CvRect r;
          ifstream ifs(regionsFile);
          SVL_ASSERT(!ifs.fail());
          while (1) {
              ifs >> r.x >> r.y >> r.width >> r.height;
              if (ifs.fail()) break;
              svlObject2d o(r);
              o.pr = detector.evaluateModel(images, r);
              o.pr = 1.0 / (1.0 + exp(-o.pr));
              o.name = string(objectName);
              objects.push_back(o);
          }
          ifs.close();
      } else if (bBaseScaleOnly) {
	  vector<CvPoint> locations;
	  if ((subRegion.width > 0) && (subRegion.height > 0)) {
	    detector.createWindowLocations(subRegion.width, subRegion.height, locations);
	    for (vector<CvPoint>::iterator it = locations.begin(); it != locations.end(); ++it) {
	      it->x += subRegion.x;
	      it->y += subRegion.y;
	    }
	  } 
	  else {
	    detector.createWindowLocations(images[0]->width, images[0]->height, locations);
	  }
	  detector.classifyImage(images, locations, &objects);
	} 
      else if ((subRegion.width > 0) && (subRegion.height > 0)) {
	detector.classifySubImage(images, subRegion, &objects);
      } 
      else if (cacheFolder) {
	if (locations_map.empty())
	  detector.createAllWindowLocations(images[0]->width, images[0]->height, locations_map);
	detector.classifyImage(images, locations_map, objects_map);
      } else {
	detector.classifyImage(images, &objects);
      }
      svlCodeProfiler::toc(classifyHandle);

      if (cacheFolder) {
	// output the detections found and along the way flatten them to make the rest of the code work as before
	double scale = 1.0;
	for (unsigned i = 0; i < objects_map.size(); i++) {
	  map<double, svlObject2dFrame>::iterator it = objects_map.find(scale);
	  if (it == objects_map.end())
	    SVL_LOG(SVL_LOG_FATAL, "scale " << scale << " is missing from objects");

	  svlObject2dFrame &frame = it->second;
	  writeShortCacheObject2dFrame(cacheFolder, strBaseName(loader.getFrameBasename(index)).c_str(), i, frame);

	  scaleObject2dFrame(frame, scale);
	  objects.reserve(objects.size() + frame.size());
	  for (unsigned j = 0; j < frame.size(); j++) {
	    objects.push_back(frame[j]);
	  }
	  scale *= svlSlidingWindowDetector::DELTA_SCALE;
	}
      }

      // non maximal suppression
      if (bUseNonmax) {
	int total = objects.size();
	int removed = nonMaximalSuppression(objects, 0.25, 0.25, 0.64);
	SVL_LOG(SVL_LOG_VERBOSE, removed << " out of " << total << " non-maximal objects suppressed");     
      }

      if (bDisplayImage) {
	IplImage *colourImage = cvCreateImage(cvSize(images[0]->width, images[0]->height),
					      IPL_DEPTH_8U, 3);
	if (images[0]->nChannels == 1) {
	  cvCvtColor(images[0], colourImage, CV_GRAY2RGB);
	} else if (images[0]->nChannels == 3) {
	  cvCopy(images[0], colourImage);
	}
	if (resultsImage != NULL) {	
	  cvReleaseImage(&resultsImage);
	}
       
	for (unsigned i = 0; i < objects.size(); i++) {
	  cvRectangle(colourImage, cvPoint((int)objects[i].x, (int)objects[i].y),
		      cvPoint((int)(objects[i].x + objects[i].w - 1), 
			      (int)(objects[i].y + objects[i].h - 1)),
		      CV_RGB(0, (unsigned char)(255 * (objects[i].pr - threshold) / (1.0 - threshold)), 0), 3, 8);
	  cvRectangle(colourImage, cvPoint((int)objects[i].x, (int)objects[i].y),
		      cvPoint((int)(objects[i].x + objects[i].w - 1), 
			      (int)(objects[i].y + objects[i].h - 1)),
		      CV_RGB(255, 255, 255), 1);
	}
       
	cvRectangle(colourImage, cvPoint(subRegion.x, subRegion.y),
		    cvPoint(subRegion.x + subRegion.width - 1, subRegion.y + subRegion.height - 1), 
		    CV_RGB(0, 0, 255), 1);
       
	if (annotatedImages[index % annotatedImages.size()] != NULL) {
	  cvReleaseImage(&annotatedImages[index % annotatedImages.size()]);
	}
	annotatedImages[index % annotatedImages.size()] = colourImage;
	resultsImage = combineImages(annotatedImages);
	cvShowImage(WINDOW_NAME, resultsImage);
	int c = cvWaitKey(index != numImages - 1 ? 100 : 0);
	//int c = cvWaitKey(0);
       
	if (c == (int)'s') {
	  stringstream filename;
	  filename << "classifyImage" << index << ".jpg";
	  cvSaveImage(filename.str().c_str(), resultsImage);
	}
      }

      // output objects
      if (groundTruth) {
	// will run non-max suppression on objects!
	analyzer.accumulateScoresForFrame(strFilename(loader.getFrameBasename(index)), objects);
      }
      if (outputFilename) {
	if (outputThreshold > threshold)
	  removeBelowProbability(objects, outputThreshold);
	writeObject2dFrame(ofs, objects, strFilename(loader.getFrameBasename(index)).c_str());
      }
     
      // free image
      for (unsigned i = 0; i < images.size(); i++) {
	cvReleaseImage(&images[i]);
      }
     
    }//image names in image sequence loop
 
  if (outputFilename) {
    ofs << "</Object2dSequence>" << endl;
    ofs.close();
  }

  if (groundTruth && prFilename) {
    analyzer.createPRcurves();
    analyzer.writePRcurves(prFilename);
  }
  if (groundTruth && summaryFilename) {
    analyzer.writeSummaryScores(summaryFilename);
  }

  // free results
  if (resultsImage != NULL) {
    cvReleaseImage(&resultsImage);
  }
  for (unsigned i = 0; i < annotatedImages.size(); i++) {
    if (annotatedImages[i] != NULL) {
      cvReleaseImage(&annotatedImages[i]);
    }
  }
 

  svlCodeProfiler::toc(svlCodeProfiler::getHandle("main"));
  svlCodeProfiler::print(cerr);
 
  return 0;
}



SVL_AUTOREGISTER_CPP( HogWrapper , svlFeatureExtractor, HogWrapper);
