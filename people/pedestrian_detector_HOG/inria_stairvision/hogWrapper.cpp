//Author: Ian Goodfellow <ia3n@cs.stanford.edu>
//File: hogWrapper.cpp

#include "hogWrapper.h"

HogWrapper::HogWrapper() : _validChannel(0)
{
  _hog = new cv::HOGDescriptor();
  assert(_hog);
}

HogWrapper::HogWrapper(unsigned validChannel) : _validChannel(validChannel)
{
  _hog = new cv::HOGDescriptor();
  assert(_hog);
}


void HogWrapper::extract(const vector<CvPoint> &locations,
		       const vector<svlDataFrame*> &frames,
		       vector< vector<double> > &output,
		       bool sparse,
		       int outputOffset) const
{
  unsigned nf = _hog->getDescriptorSize();

  if (output.size() < locations.size())
    output.resize(locations.size(), vector<double>(nf));

  svlImageFrame * imgFrame = dynamic_cast<svlImageFrame *>(frames.at(_validChannel));

  if (!imgFrame)
    SVL_LOG(SVL_LOG_FATAL, "Channel "<<_validChannel<<" is not an image channel, yet was passed to Haar feature extractor");

  IplImage * img = imgFrame->image;


  cv::Mat mat(img);

  cv::Vector<float> results_cv;

  cv::Vector<cv::Point> locations_cv(locations.size());

  for (unsigned i = 0; i < locations.size(); i++)
    locations_cv[i] = cv::Point(locations[i].x, locations[i].y);
  

  _hog->compute(mat, results_cv, cv::Size(0,0), cv::Size(0,0), locations_cv);



  //<debug>
  /*
  cv::HOGDescriptor hog(cv::Size(64,128), cv::Size(16,16), cv::Size(8,8), cv::Size(8,8), 9);


  char * hogp = (char *) & hog;
  char * _hogp = (char *) &(*_hog);

  cout << "Side by side: " << endl;
  for (unsigned i = 0; i < sizeof(cv::HOGDescriptor); i++)
    {
      cout << (int) hogp[i] << " " << (int) _hogp[i];
      if ((int) hogp[i] != (int) _hogp[i])
	cout << " !!!!!!!!";
      cout << endl;
    }

assert(hog.winSize == _hog->winSize);
  assert(hog.blockSize == _hog->blockSize);
  assert(hog.blockStride == _hog->blockStride);
  assert(hog.cellSize == _hog->cellSize);
  assert(hog.nbins == _hog->nbins);
  assert(hog.derivAperture == _hog->derivAperture);
  assert(hog.winSigma == _hog->winSigma);
  assert(hog.histogramNormType == _hog->histogramNormType);
  assert(hog.L2HysThreshold == _hog->L2HysThreshold);
  assert(hog.gammaCorrection == _hog->gammaCorrection);
  assert(hog.svmDetector.size() == _hog->svmDetector.size());
  for (unsigned i = 0; i < hog.svmDetector.size(); i++)
    assert(hog.svmDetector[i] == _hog->svmDetector[i]);

  char * p;
#define PRINT(x) cout << #x << endl; p = (char *) & x;		\
					    for (unsigned i = 0; i < sizeof(x); i++) \
					      cout << (int) p[i] << endl; \ 

  PRINT(hog.winSize);
  PRINT(hog.blockSize);
  PRINT(hog.blockStride);
  PRINT(hog.cellSize);
  PRINT(hog.nbins);
  PRINT(hog.derivAperture);
  PRINT(hog.winSigma);
  PRINT(hog.histogramNormType);
  PRINT(hog.L2HysThreshold);
  PRINT(hog.gammaCorrection);
  PRINT(hog.svmDetector);




  cv::Mat rimg(128,64,CV_8U);
  
  cv::Vector<cv::Point> locations2;

  cv::RNG rng = cv::theRNG() ;

  rng.fill(rimg, cv::RNG::UNIFORM, cv::Scalar::all(0), cv::Scalar::all(256));

  locations2.push_back(cv::Point(0,0));

  cv::Vector<float> descriptors;

  //_hog->compute(mat, results_cv, cv::Size(), cv::Size(), locations2);

  */

  /*
  cout  << "Nonzero feature values: " << endl;
  for (unsigned i = 0; i < results_cv.size(); i++)
    {
      if (results_cv[i] != 0)
	cout << results_cv[i] << endl;
    }

  */
  //exit(-1);

  assert(results_cv.size() == locations.size() * nf);

  for (unsigned i = 0; i < locations.size(); i++)
    {
      unsigned start = i * nf;
      unsigned finish = (i+1) * nf;


      for (unsigned j = start; j < finish; j++)
	{
	  assert(j < results_cv.size());
	  assert(i < output.size());
	  assert(j-start < output[i].size());
	  output[i][j-start] = results_cv[j];
	}
    }
}

bool HogWrapper::writeOut(ofstream &ofs)
{
  ofs << "<svlFeatureExtractor id=\"HogWrapper\" ";

  ofs << " version=\"1.0\" ";
  ofs << " width=\"" << _hog->winSize.width << "\" ";
  ofs << " height=\"" << _hog->winSize.height << "\" ";
  ofs << " blockWidth=\"" << _hog->blockSize.width << "\" ";
  ofs << " blockHeight=\"" << _hog->blockSize.height << "\" ";
  ofs << " blockStrideX=\"" << _hog->blockStride.width << "\" ";
  ofs << " blockStrideY=\"" << _hog->blockStride.height << "\" ";
  ofs << " nBins=\"" << _hog->nbins << "\" ";
  ofs << " derivAperture=\"" << _hog->derivAperture<<"\" ";
  ofs << " winSigma=\"" << _hog->winSigma<<"\" ";
  ofs << " histogramNormType=\"" << _hog->histogramNormType << "\" ";
  ofs << " L2HysThreshold=\"" << _hog->L2HysThreshold << "\" ";
  ofs << " gammaCorrection=\""<< (bool) _hog->gammaCorrection<<"\" ";
  ofs << " validChannel=\""<<_validChannel<<"\" ";
  ofs << " cellWidth=\""<<_hog->cellSize.width<<"\" ";
  ofs << " cellHeight=\""<<_hog->cellSize.height<<"\" ";


  ofs << "> " << endl;

  ofs << "</svlFeatureExtractor>" << endl;

  return !ofs.fail();
}

bool HogWrapper::load(XMLNode &root)
{
  if (root.isEmpty())
    {
      SVL_LOG(SVL_LOG_WARNING, "Haar Feature Extractor XML root is empty");
      return false;
    }

  if (!root.getName())
    {
      SVL_LOG(SVL_LOG_WARNING, "HogWrapper::load given XML file with root with no name.");
      return false;
    }

  if (strcmp(root.getName(), "svlFeatureExtractor") != 0)
    {
      SVL_LOG(SVL_LOG_WARNING, "Attempt to read HogWrapper feature extractor from XML node that is not a feature extractor. Expected \"svlFeatureExtractor\", received \""<<root.getName());
      return false;
    }

  //todo-- add error checking

  _windowSize.width = atoi(root.getAttribute("width"));
  _windowSize.height = atoi(root.getAttribute("height"));

  SVL_LOG(SVL_LOG_MESSAGE, "HogWrapper loaded with size "<<_windowSize.width << "x" << _windowSize.height);

  _validChannel = atoi(root.getAttribute("validChannel"));

  cv::Size blockSize;

  blockSize.width = atoi(root.getAttribute("blockWidth"));
  blockSize.height = atoi(root.getAttribute("blockHeight"));
  
  cv::Size blockStride;

  blockStride.width = atoi(root.getAttribute("blockStrideX"));
  blockStride.height = atoi(root.getAttribute("blockStrideY"));

  int nbins = atoi(root.getAttribute("nbins"));

  int derivAperture = atoi(root.getAttribute("derivAperture"));

  double winSigma = atof(root.getAttribute("winSigma"));

  int histogramNormType = atoi(root.getAttribute("histogramNormType"));

  double L2HysThreshold = atoi(root.getAttribute("L2HysThreshold"));

  bool gammaCorrection = atoi(root.getAttribute("gammaCorrection"));

  cv::Size windowSize(_windowSize);

  cv::Size cellSize;

  cellSize.width = atoi(root.getAttribute("cellWidth"));
  cellSize.height = atoi(root.getAttribute("cellHeight"));

  _hog = new cv::HOGDescriptor(windowSize, blockSize, cellSize, blockStride, nbins, derivAperture, winSigma, histogramNormType, L2HysThreshold, gammaCorrection);

  _hog = new cv::HOGDescriptor(windowSize, blockSize, cellSize, blockStride, nbins);

  cv::HOGDescriptor hog2(windowSize, blockSize, cellSize, blockStride, nbins);

  assert(_hog);


  cv::HOGDescriptor hog(cv::Size(64,128), cv::Size(16,16), cv::Size(8,8), cv::Size(8,8), 9);
  
  /*
  
  char * hogp = (char *) & hog;
  char * _hogp = (char *) &(*_hog);
  
  cout << "Side by side: " << endl;
  for (unsigned i = 0; i < sizeof(cv::HOGDescriptor); i++)
    {
      cout << (int) hogp[i] << " " << (int) _hogp[i];
      if ((int) hogp[i] != (int) _hogp[i])
	cout << " !!!!!!!!";
      cout << endl;
    }


  assert(hog.winSize == _hog->winSize);
  assert(hog.blockSize == _hog->blockSize);
  assert(hog.blockStride == _hog->blockStride);
  assert(hog.cellSize == _hog->cellSize);
  assert(hog.nbins == _hog->nbins);
  assert(hog.derivAperture == _hog->derivAperture);
  assert(hog.winSigma == _hog->winSigma);
  assert(hog.histogramNormType == _hog->histogramNormType);
  assert(hog.L2HysThreshold == _hog->L2HysThreshold);
  assert(hog.gammaCorrection == _hog->gammaCorrection);
  assert(hog.svmDetector.size() == _hog->svmDetector.size());
  for (unsigned i = 0; i < hog.svmDetector.size(); i++)
    assert(hog.svmDetector[i] == _hog->svmDetector[i]);

  cout << "(right after construction)" << endl;
  */

  return true;
}

svlFeatureExtractor* HogWrapper::getPrunedExtractor
     (const vector<bool> &featureUsedBits) const
{
  SVL_LOG(SVL_LOG_WARNING, "cv::HOGDescriptor can't be pruned, so HogWrapper does not support pruning");
  return NULL;
}

svlFeatureExtractor * HogWrapper::clone() const
{
  HogWrapper * c = new HogWrapper(*this);
  assert(c);
  svlFeatureExtractor * rval = dynamic_cast<svlFeatureExtractor *>(rval);

  return rval;
}

string HogWrapper::summary() const
{
  stringstream s;
  s << "["<<numFeatures()<<" HOG features]";
  return s.str();
}

void HogWrapper::setSize(CvSize size)
{
  assert(false); //not yet supported-- will need to copy all fields of cv object to new one
  
  svlFeatureExtractor::setSize(size);
}

HogWrapper::~HogWrapper()
{
  //svlSmartPointer handles deletion of _hog
}

unsigned HogWrapper::numFeatures() const
{
  /*
  cout << "inside numFeatures call" << endl;


  cout << _hog->winSize.width << " " << _hog->winSize.height << endl;
  cout << _hog->blockSize.width << " " << _hog->blockSize.height << endl;
  cout << _hog->cellSize.width << " " << _hog->cellSize.height << endl;
  cout << "nbins" << endl;
  cout << _hog->nbins << endl;

  cout <<  _hog->derivAperture << endl;
  cout <<  _hog->winSigma << endl;
  cout << _hog->histogramNormType << endl;
  cout <<  _hog->L2HysThreshold << endl;
  cout << _hog->gammaCorrection << endl;

  cout << "svm: " << endl;
  for (unsigned i = 0; i < _hog->svmDetector.size(); i++)
    cout << _hog->svmDetector[i] << endl;
  cout << "/svm" << endl;
  */


  return _hog->getDescriptorSize();
}

HogWrapper::HogWrapper(const HogWrapper & o)
{
  _validChannel = o._validChannel;

  assert(false);  //_hog = new cv::HOGDescriptor();
  //the cv::HOGDescriptor copy constructor doesn't seem to work
}

const HogWrapper & HogWrapper::operator=(const HogWrapper & o)
{
  assert(false); //unimplemented
}


