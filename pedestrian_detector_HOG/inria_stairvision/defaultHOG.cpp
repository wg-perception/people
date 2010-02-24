//Author: Ian Goodfellow
//File: defaultHog.cpp
//Description: utility for making a file containing the default HOG detector


#include "svlVision.h"
#include "hogWrapper.h"

void usage()
{
  cerr << SVL_USAGE_HEADER << endl;
  cerr << "USAGE: ./defaultHOG -ch <channel> -o <filepath>" << endl;
  cerr << "Writes out a file to <filepath> for a HOG descriptor with default parameters and configured to act on channel <channel>" << endl;
}

int main(int argc, char ** argv)
{
  const char * filepath = NULL;
  int channel  = -1;

  SVL_BEGIN_CMDLINE_PROCESSING(argc,argv)
    SVL_CMDLINE_STR_OPTION("-o",filepath)
    SVL_CMDLINE_INT_OPTION("-ch",channel)
  SVL_END_CMDLINE_PROCESSING(usage())

    if (!filepath)
      {
	usage();
	return -1;
      }
  
  if (channel < 0)
    {
      usage();
      return -1;
    }
  
  HogWrapper * hog = new HogWrapper(channel);
  assert(hog);
  assert(hog->write(filepath));
  
  
  return 0;
}
