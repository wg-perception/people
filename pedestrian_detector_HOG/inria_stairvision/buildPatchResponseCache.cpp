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
** FILENAME:    buildPatchResponseCache.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Olga Russakovsky <olga@cs.stanford.edu>
** DESCRIPTION:
**  Build a cache of patch responses.
**
*****************************************************************************/

#define NUM_THREADS 0

#include "hogWrapper.h"
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/types.h>
#include <sstream>

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

using namespace std;

// Thread args
struct ProcessFileArgs {
  // idx is the file number within the directory; sample_idx is the number of
  // the output file
  // loader is a vector of NUM_THREADS svlImageLoader objects. Each thread should only use loader[tid] to avoid race conditions on loader's edge extractor object
  ProcessFileArgs(  const char *outD,
		    const vector<svlFeatureExtractor *> & dict,  unsigned idx, const vector<svlImageLoader *> & loader,
		    unsigned sample_idx) :
      outputDir(outD),  
      dictionary(dict), idx(idx), loader(loader), sample_idx(sample_idx) {}



  const char *outputDir;

  const vector<svlFeatureExtractor *> & dictionary;
  unsigned idx;
  const vector<svlImageLoader *> & loader;
  unsigned sample_idx;
};

// Thread function
void *processFile(void *voidArgs, unsigned tid) {



  //Copy arguments out of the helper struct
  ProcessFileArgs *args = (ProcessFileArgs*) voidArgs;
  const char *outputDir = args->outputDir;
  const vector<svlFeatureExtractor *> &  dictionary = args->dictionary;
  unsigned idx = args->idx;
  unsigned sample_idx = args->sample_idx;
  const vector<svlImageLoader *> & loader = args->loader;

  vector<IplImage *> images;

  if (!loader.at(tid)->getFrame(idx, images))
    return NULL;
            

  //Discard nearly uniform examples
  /*
  for (unsigned i = 0; i < images.size(); i++)
    {
      CvScalar mean;
      CvScalar sdev;
      
      cvAvgSdv(images[i], &mean, &sdev);

      SVL_LOG(SVL_LOG_DEBUG, "standard deviation: "<<sdev.val[0]);

      if ( (images[i]->depth==IPL_DEPTH_8U && sdev.val[0] < 5.0) || (images[i]->depth==IPL_DEPTH_32F && sdev.val[0] < 0.01)) //TODO-- figure out why svlPatchDefinition::patchValueHelper sometimes has windowNorm come out <= 0 even though patch is not uniform. Is it just numerical error?
	{
	  //stringstream filepath;
	  //filepath << "output_" << rand() << ".jpg";
	  //cvSaveImage(filepath.str().c_str(), images[i]);
	  return NULL;
	}
	}*/

  // process image
  vector<double> v;


  dictionary.at(tid)->windowResponse(images, v);



  for (unsigned i = 0; i < v.size(); i++)
    {
      //if (v[i] == FLT_MAX || v[i] == - FLT_MAX)
      //{
      //  cvSaveImage("/tmp/flt_max.jpg",images[0]);
      //}
      
      if (v[i] > 10e30)
	{
	  cout << v[i] << endl;
	  cout << "buildPatchResponseCache override" << endl;
	  v[i] = 0;
	}

      if (v[i] < -10e30)
	{
	  cout << v[i] << endl;
	  cout << "buildPatchResponseCache override" << endl;
	  v[i] = 0;
	}

      assert(v[i] != FLT_MAX);
      assert(v[i] != - FLT_MAX);
      assert(v[i] < 0.9 * FLT_MAX);
      assert(v[i] > 0.9 * - FLT_MAX);
    }

 
  svlWriteExampleToCache(outputDir, sample_idx, v);
                       

  // Free memory
  for (unsigned i = 0; i < images.size(); i++)
    cvReleaseImage(&(images[i]));



  delete args;


  return NULL;
}

void processBinaryFile(std::ostream& ofs, const svlFeatureExtractor * dictionary,
                       unsigned idx, svlImageLoader& loader) {
  vector<IplImage *> images;

  if (!loader.getFrame(idx, images))
    return ;

  // process image
  vector<double> v;
  dictionary->windowResponse(images,v);

  // write feature vector
  if (ofs.fail()) {
    SVL_LOG(SVL_LOG_WARNING, "...failed on index " << idx);
  } else {    
    svlWriteExampleToBinaryCache(ofs, v);
  }
            
  // Free memory
  for (unsigned i = 0; i < images.size(); i++)
    cvReleaseImage(&(images[i]));
}


void usage()
{
    cerr << SVL_USAGE_HEADER << endl;
    cerr << "USAGE: ./buildPatchResponseCache [OPTIONS] (<image dir> | <image sequence>) <output dir> <feature extractor>" << endl;
    cerr << "OPTIONS:" << endl
	 << "  -maxImages <num>   :: maximum training images to cache (default: 10000)" << endl
 	 << "  -binary            :: creates binary cache file named <output dir>.bin" << endl
	 << "  -parallel <id> <ct>:: used for distributed jobs on the cluster." << endl
	 << "  -startIndex        :: starts numbering from this index instead of from 0" << endl
	 << "  -augment           :: augments the existing cache dir; overrides -startIndex " << endl
	 << "                        (same as using -startIndex [curr # files in cache dir])" << endl
	 << SVL_STANDARD_OPTIONS_USAGE << endl
	 << "  Note: both -augment and -startIndex make sure the TOTAL number of files in the " << endl
	 << "        cache dir doesn't exceed -maxImages, not just the number of files added" << endl
	 << "        in the current run." << endl
	 << endl;
}


int main(int argc, char *argv[])
{
  // svlAutoregsvlFeatureExtractorFileCheckersvlFeatureExtractor.debug();
  captureOpenCVerrors();
  svlCodeProfiler::tic(svlCodeProfiler::getHandle("buildPatchResponseCache"));

  // read commandline parameters
  const int NUM_REQUIRED_PARAMETERS = 3;
  
  int maxImages = 10000;
  bool binary = false;
  int  parallelId = 0;
  int  parallelCount = 0;
  int startIndex = 0;
  bool augment = false;

  SVL_BEGIN_CMDLINE_PROCESSING(argc, argv)
    SVL_CMDLINE_INT_OPTION("-maxImages", maxImages)
    SVL_CMDLINE_BOOL_OPTION("-binary", binary)
    SVL_CMDLINE_OPTION_BEGIN("-parallel", ptr)
      parallelId = atoi(ptr[0]);
      parallelCount = atoi(ptr[1]);
      if (parallelId < 0 || parallelId >= parallelCount) {
        SVL_LOG(SVL_LOG_FATAL, "illegal id number " << parallelId);
      } else if (parallelCount < 0) {
        SVL_LOG(SVL_LOG_FATAL, "illegal job count " << parallelCount);
      }
    SVL_CMDLINE_OPTION_END(2)
    SVL_CMDLINE_INT_OPTION("startIndex", startIndex)
    SVL_CMDLINE_BOOL_OPTION("augment", augment)
  SVL_END_CMDLINE_PROCESSING(usage());
    
  if (SVL_CMDLINE_ARGC != NUM_REQUIRED_PARAMETERS) {
    usage();
    return -1;
  }

  const char *imageDir = SVL_CMDLINE_ARGV[0];
  const char *outputDir = SVL_CMDLINE_ARGV[1];
  const char *extractorFilename = SVL_CMDLINE_ARGV[2];

  svlImageLoader loader;

  // read dictionary
  const svlFeatureExtractor *  dictionary = svlFeatureExtractorFactory().load(extractorFilename);
    


  if (!dictionary)
    SVL_LOG(SVL_LOG_FATAL, "Could not load "<<extractorFilename);

  SVL_LOG(SVL_LOG_MESSAGE, dictionary->numFeatures() << " features in the feature extractor");

  int width = dictionary->windowWidth();
  int height = dictionary->windowHeight();
  
  // will be compared to the maxImages parameter
  int numImages = 0;
  if (startIndex > 0 || augment)
    numImages = svlDirectoryListing(outputDir).size();
  
  if (augment)
    startIndex = numImages;

  if (strExtension(imageDir).compare("xml") == 0) {  
    loader.readImageSeq(imageDir);
  } else {
    loader.readDir(imageDir);
  }

  loader.setResizeSize(width,height);
  
  if (!binary) {
    // ------ Output ASCII cache files ------
    
    // TODO: how many threads?
    svlThreadPool threadPool(NUM_THREADS);
    threadPool.start();
    
    vector<svlImageLoader *> loaderSet;
    vector<svlFeatureExtractor *> dictionarySet;
    for (unsigned i = 0; i < NUM_THREADS; i++)
      {

	loaderSet.push_back(new svlImageLoader(loader));
	svlFeatureExtractor * clone = dictionary->clone();
	assert(clone);
	dictionarySet.push_back(clone);
      }

    if (NUM_THREADS == 0)
      {
	loaderSet.push_back(&loader);
	dictionarySet.push_back((svlFeatureExtractor * )dictionary);
      }
    


    // i is the index of the image we're processing, numImages is the number of
    // images in the cache directory currently, i+startIndex is the name of the file
    // we're about to output
    for (unsigned i = 0; i < loader.numFrames() && numImages < maxImages; i++, numImages++) {
      //if (parallelCount > 0) { // UNTESTED
      //if ((i % parallelCount) != parallelId) continue ;  // UNTESTED
      //}  // UNTESTED
      
      
      threadPool.addJob(processFile,
			new ProcessFileArgs(outputDir, dictionarySet, i, loaderSet, i + startIndex));
    }
    // TODO: make this contingent on success?
    
    threadPool.finish();
    
    for (unsigned i = 0; i < NUM_THREADS; i++)
      delete loaderSet[i];

    for (unsigned i = 0; i < NUM_THREADS; i++)
      delete dictionarySet[i];

    delete dictionary;

  } else {
    // ------ Output binary cache file ------
    if (augment) {
      SVL_LOG(SVL_LOG_FATAL, "-augment flag and -binary flag are incompatible at present.");
    }
    
    // erases the possible "/" at the end of the directory name
    string outputDirStr = string(outputDir);
    int pos = outputDirStr.find_last_not_of("/");
    outputDirStr = outputDirStr.substr(0, pos+1);
    
    std::string jobId = "";
    if (parallelCount > 0) {
      jobId = "." + toString(parallelId);
    }
    std::string fileName = outputDirStr+jobId+".bin";
    std::fstream ofs(fileName.c_str(), std::ios::out|std::ios::binary);
    
    for (unsigned i = 0; i < loader.numFrames() && numImages < maxImages; i++, numImages++) {
      if (parallelCount > 0) {
	if ((i % unsigned(parallelCount)) != unsigned(parallelId)) continue ;
      }
      processBinaryFile(ofs, dictionary, i, loader);
    }
  }
  
  svlCodeProfiler::toc(svlCodeProfiler::getHandle("buildPatchResponseCache"));
  svlCodeProfiler::print(cerr);
  
  return 0;
}

SVL_AUTOREGISTER_CPP( HogWrapper , svlFeatureExtractor, HogWrapper);
