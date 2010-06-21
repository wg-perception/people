/** STAIR VISION LIBRARY
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
** FILENAME:    extractorCat.cpp
** AUTHOR(S):   Ian Goodfellow <ia3n@cs.stanford.edu>
** DESCRIPTION:
**   Application for concatenating feature extractor files
*****************************************************************************///

#include "svlVision.h"


void usage()
{
  cout << "./extractorCat -o <outputFilename.xml> inputFile1 inputFile2 inputFile3 ... " << endl;
  cout << "inputFiles describe feature extractors. Produces outputFile that defines a feature extractor that extracts all of the features from the inputFiles. This is a deep copy so it is safe to remove/delete the input files if you don't need them otherwise." << endl;
}

int main(int argc, char ** argv)
{
  
  vector<svlFeatureExtractor *> extractors;

  const char * fileOut = NULL;

  for (int i = 1; i < argc; i++)
    {
      if (!strcmp(argv[i],"-o"))
	{
	  i++;

	  if (i == argc)
	    {
	      usage();
	      SVL_LOG(SVL_LOG_FATAL, "Argument list ends after -o");
	    }

	  fileOut = argv[i];
	}
      else
	{
	  string filename = string(argv[i]);
	  extractors.push_back(svlFeatureExtractorFactory().load(filename.c_str()));
	  if (! extractors.back())
	    {
	      usage();
	      SVL_LOG(SVL_LOG_FATAL, "Could not open file \""<<filename<<"\"");
	    }
	  
	}
    }


  if (!fileOut)
    {
      usage();
      SVL_LOG(SVL_LOG_FATAL,"The -o argument is required");
    }

  svlCompositeFeatureExtractor * result = new svlCompositeFeatureExtractor(extractors);

  assert(result);


  result->write(fileOut);


  return 0;
}
