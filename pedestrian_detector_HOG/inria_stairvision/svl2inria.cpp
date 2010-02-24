//Author: Ian Goodfellow <goodfellow@willowgarage.com> (until Sep '09) <ia3n@cs.stanford.edu> (forever)
//Description: An application for converting an svl dataset to an inria dataset

#include "svlVision.h"
#include <sstream>
using namespace std;

void usage()
{
  cerr << SVL_USAGE_HEADER << endl;
  cerr << "USAGE: ./svl2inria -is <svl image sequence> -il <svl labels> -od <directory of inria data to create>" << endl;
  cerr << "Pass -t for test set, otherwise assumes train. (the INRIA format uses a different size for test positives than train positives) " << endl;
  
}

vector<IplImage *> getNegativeImages(svlLabeledFrame & f);
vector<IplImage *> getPositiveImages(svlLabeledFrame & f, int w, int h, int xpad, int ypad);//for each positive example, grows the smaller dimension until it is proportional to a (w+2*xpad)x(h+2*ypad) rectangle. if this requires it to extend outside the image, fills in the extra pixels with black (makes sense for infrared, maybe not for other sensors). 
void removeImagesSmallerThan(vector<IplImage *> & images, int minWidth, int minHeight);
vector<IplImage *> getImagesFromBoundingBoxes(const IplImage * im, const svlObject2dFrame & labels);
vector<IplImage *> getImagesFromBoundingBoxesAllowingOutOfBounds(const IplImage * im, const svlObject2dFrame & boxes);  //pixels outside the image are treated as black
string makeDirectories(const char * outputDir); // returns outputDir as a full path
void writeImages(const vector<IplImage *> & images, const string & baseDir, bool isPos);
bool intersects(const svlObject2d & a, const svlObject2d & b);
svlObject2dFrame growToProportion(const svlObject2dFrame & input, double wOverh);
svlObject2dFrame rects2objects(const vector<CvRect> & rects);
void scaleImages(vector<IplImage *> & v,int targW,int targH);

int main(int argc, char ** argv)
{
  const char * inputSeq = NULL;  
  const char * inputLab = NULL;
  const char * outputDir = NULL;
  bool testSet = false;

  SVL_BEGIN_CMDLINE_PROCESSING(argc,argv)
    SVL_CMDLINE_STR_OPTION("-is",inputSeq)
    SVL_CMDLINE_STR_OPTION("-il",inputLab)
    SVL_CMDLINE_STR_OPTION("-od",outputDir)
    SVL_CMDLINE_BOOL_OPTION("-t",testSet)
  SVL_END_CMDLINE_PROCESSING(usage())
  
    if (!inputSeq || !inputLab || !outputDir)
      {
	usage();
	return -1;
      }

  int width = 64;
  int height = 128;
  int xpad = 0;
  int ypad = 0;

  if (testSet)
    {
      xpad = ypad = 3;
    }
  else
    {
      xpad = ypad = 16;
    }
  

  svlLabeledSequence s;
  s.load(inputSeq, inputLab);

  string fullPath = makeDirectories(outputDir);

  for (unsigned i = 0; i < s.numImages(); i++)
    {
      //cout << "Processing frame "<<i<<endl;
      svlLabeledFrame l = s.frame(i);

      assert(l.image);

      if (!l.labels)
	{
	  l.labels = new svlObject2dFrame;
	  cerr << "WARNING: Adding empty label to unlabeled frame "<<s[i]<<endl;
	  continue;
	}

      vector<IplImage *> negImages = getNegativeImages(l);
      removeImagesSmallerThan(negImages,width,height);
      writeImages(negImages, fullPath, 0);
      vector<IplImage *> posImages = getPositiveImages(l,width,height,xpad,ypad);
      writeImages(posImages, fullPath, 1);
    }

  return 0;
} 

vector<IplImage *> getNegativeImages(svlLabeledFrame & f)
{
  //Negative images may be any size but may not contain any positives at all
  //The INRIA training code will extract random portions of them to make
  //negative examples

  //Most SVL datasets consist largely of images containing positives; the challenge
  //is to correctly localize them

  //Thus we need to make negative images by cropping out positives

  assert(f.image);
  assert(f.labels);//the main function should have filtered out frames with no label data structure associated (this is different from no labels)

  vector<IplImage *> rval;

  if (f.labels->size() == 0) //if there are no labels, then this whole frame can be used for negative examples
    {
      rval.push_back(cvCloneImage(f.image));
      return rval;
    }

  svlObject2dFrame candidates;
  
  //Initialize candidates with one rect filling the whole image
  candidates.push_back(svlObject2d(0,0,f.image->width,f.image->height));


  //for l in labels
  for (unsigned l = 0; l < f.labels->size(); l++)
    {
      //cout << "\tLabel "<<l<<" of "<<f.labels->size()<< endl;
      svlObject2d label = f.labels->operator[](l);


      //for c in candidates
      for (unsigned c = 0; c < candidates.size(); c++)
	{
	  //cout << "\t\tCandidate "<<c<<" of "<<candidates.size()<<endl;
	  //if l intersects c
	  if (intersects(candidates[c],label) > 0)
	    {
	      //svlLabelVisualizer v("Intersect?");
	      //v.addLabel(candidates[c],SVL_DETECTION);
	      //v.addLabel(label, SVL_TRUTH);
	      //v.enableWindow();
	      //cvWaitKey(0);

	      //break c into part that is above, below, left, right of l;

	      //left
	      if (candidates[c].x < label.x)
		{
		  candidates.push_back(svlObject2d(candidates[c].x, candidates[c].y, label.x - candidates[c].x, candidates[c].h));
		  assert(candidates.back().w > 0);
		  assert(candidates.back().h > 0);
		}

	      //right
	      if (candidates[c].x + candidates[c].w > label.x + label.w)
		{
		  candidates.push_back(
				       svlObject2d(label.x+label.w, 
						   candidates[c].y,
						   candidates[c].x+candidates[c].w - label.x-label.w, 
						   candidates[c].h));
		  assert(candidates.back().w > 0);
		  assert(candidates.back().h > 0);
		}

	      //top
	      if (candidates[c].y < label.y)
		{
		  candidates.push_back(svlObject2d(candidates[c].x, candidates[c].y, candidates[c].w, label.y-candidates[c].y));
		  assert(candidates.back().w > 0);
		  assert(candidates.back().h > 0);
		}

	      //bottom
	      if (candidates[c].y + candidates[c].h > label.y + label.h)
		{
		  candidates.push_back(svlObject2d(label.x, label.y + label.h, label.w, candidates[c].y+candidates[c].h - label.y-label.h));
		  assert(candidates.back().w > 0);
		  assert(candidates.back().h > 0);
		}

	      candidates.erase(candidates.begin()+c);
	      
	      c--; //prevent c index from advancing
	    }
	}	  
    }

  removeOverlappingObjects(candidates,0.9);

  rval = getImagesFromBoundingBoxes(f.image, candidates);

  return rval;
}


vector<IplImage *> getPositiveImages(svlLabeledFrame & f, int w, int h, int xpad, int ypad)
{
  vector<IplImage *> rval;

  if (f.labels)
    {
      int targW = w + 2*xpad;
      int targH = h + 2*ypad;


      svlObject2dFrame proportionalBoxes = growToProportion(*f.labels, double(targW)/double(targH));
      rval =  getImagesFromBoundingBoxesAllowingOutOfBounds(f.image, proportionalBoxes);
      scaleImages(rval,targW,targH);
    }

  return rval;
}


void removeImagesSmallerThan(vector<IplImage *> & images, int minWidth, int minHeight)
{
  vector<IplImage *>::iterator i = images.begin();

  while (i != images.end())
    {
      if ((*i)->width < minWidth || (*i)->height < minHeight)
	i = images.erase(i);
      else
	++i;
    }
}

vector<IplImage *> getImagesFromBoundingBoxes(const IplImage * im, const svlObject2dFrame & labels)
{
  IplImage * hack = (IplImage *) im;

  vector<IplImage *> rval;

  for (unsigned i = 0; i < labels.size(); i++)
    {
      CvRect label = cvRect(labels[i].x, labels[i].y, labels[i].w, labels[i].h);

      if (label.x < 0)
	{
	  cerr << "getImagesFromBoundingBox: warning, label has x value of " << label.x << endl;
	  label.width -= label.x;
	  label.x = 0;
	}

      if (label.y < 0)
	{
	  cerr << "getImagesFromBoundingBox: warning, label has y value of " << label.y << endl;
	  label.height -= label.y;
	  label.y = 0;
	}

      if (label.x + label.width > hack->width)
	{
	  cerr << "getImagesFromBoundingBox: warning, label sticks out right of frame by "<<label.x+label.width - hack->width << " pixels" << endl;
	  label.width = hack->width - label.x;
	}

      if (label.y + label.height > hack->height)
	{
	  cerr << "getImagesFromBoundingBox: warning, label sticks out top of frame by "<<label.y+label.height - hack->height << " pixels" << endl;
	  label.height = hack->height - label.y;
	}

      if (label.width <= 0)
	{
	  cerr << "getImagesFromBoundingBox: warning, label has width of "<<label.width << " (skipping) " << endl;
	  continue;
	}

      if (label.height <= 0)
	{
	  cerr << "getImagesFromBoundingBox: warning, label has height of "<<label.height<<" (skipping) " << endl;
	  continue;
	}

      //cerr << "image: "  << hack->width << " "<< hack->height << endl;
      //cerr << "label: " << label.x << " " << label.y << " " << label.width << " " << label.height << endl;

      cvSetImageROI(hack, label);
      IplImage * output = cvCreateImage(cvSize(labels[i].w, labels[i].h),hack->depth, hack->nChannels);
      assert(hack->width > 0);
      assert(hack->height > 0);
      cvCopy(hack, output);
      cvResetImageROI(hack);
      rval.push_back(output);
    }
  return rval;
}

vector<IplImage *> getImagesFromBoundingBoxesAllowingOutOfBounds(const IplImage * im, const svlObject2dFrame & boxes)
{
  vector<CvRect> rects;
  vector<int> xEdges;//x coords of all vertical box edges
  vector<int> yEdges;//y coords of all horizontal box edges

  for (unsigned i = 0; i < boxes.size(); i++)
    {
      rects.push_back(cvRect(boxes[i].x,boxes[i].y,boxes[i].w,boxes[i].h));
      xEdges.push_back(rects.back().x);
      xEdges.push_back(rects.back().x+rects.back().width-1);
      yEdges.push_back(rects.back().y);
      yEdges.push_back(rects.back().y+rects.back().height-1);
    }

  //Guarantee that we'll never shrink the image, this way we don't need to fiddle with ROI of im
  xEdges.push_back(0);
  xEdges.push_back(im->width-1);
  yEdges.push_back(0);
  yEdges.push_back(im->height-1);

  int minX = minElem(xEdges);
  int minY = minElem(yEdges);
  int maxX = maxElem(xEdges);
  int maxY = maxElem(yEdges);

  int w = maxX-minX+1;
  int h = maxY-minY+1;

  IplImage * expanded = cvCreateImage(cvSize(w,h), im->depth, im->nChannels);

  cvZero(expanded);
  cvSetImageROI(expanded, cvRect(-minX,-minY,im->width,im->height));
  cvCopyImage(im, expanded);

  for (unsigned i = 0; i < rects.size(); i++)
    {
      rects[i].x -= minX;
      rects[i].y -= minY;
    }

  return getImagesFromBoundingBoxes(expanded, rects2objects(rects));
}

string makeDirectories(const char * outputDir)
{
  string command0 = string("rm -r -f ")+outputDir;

  FILE * output = popen(command0.c_str(), "r");

  fclose(output);


  stringstream command;

  command << "mkdir -p " << outputDir;


  output = popen(command.str().c_str(), "r");
  fclose(output);

  stringstream command2;

  command2 << "mkdir -p " << outputDir << "/pos";
  

  output = popen(command2.str().c_str(),"r");

  fclose(output);
  
  stringstream command3;

  command3 << "mkdir -p " << outputDir << "/neg";
  

  output = popen(command3.str().c_str(),"r");

  fclose(output);

  stringstream command4;

  command4 << "cd " << outputDir << "; pwd";
  
  output = popen(command4.str().c_str(),"r");


  int x;

  stringstream rval_stream;

  while ( (x = fgetc(output)) != EOF)
    {
      if (x != '\n')
	rval_stream << char(x);
    }

  return rval_stream.str();
}

void writeImages(const vector<IplImage *> & images, const string & baseDir, bool isPos)
{
  static int posCounter = 0;
  static int negCounter = 0;

  int * counter = NULL;

  assert(baseDir.size() && baseDir[0] == '/'); //baseDir should be a full path
  
  string set;
  if (isPos)
    {
      set = "pos";
      counter = & posCounter;
    }
  else
    {
      set = "neg";
      counter = & negCounter;
    }

  string listFile = baseDir+"/"+set+".lst";

  fstream ofs;
  ofs.open(listFile.c_str(), ios::out | ios::app);

  if (ofs.fail())
    {
      cerr << "Could not write to "<<listFile<< endl;
      exit(1);
    }

  string imgDir = baseDir+"/"+set;

  //cout << "Writing to "<<listFile << endl;

  for (unsigned i = 0; i < images.size(); i++)
    {
      stringstream s;
      s << imgDir << "/image_" << (*counter)++ << ".png";

      string filepath = s.str();

      ofs << filepath << endl;

      cvSaveImage(filepath.c_str(), images[i]);

      //cout << "\t" << filepath << endl;
    }

  ofs.close();
}

bool intersects(const svlObject2d & a, const svlObject2d & b)
{
  if (a.x+a.w  <= b.x +0.1)
    return false;

  if (a.x + 0.1 >= b.x+b.w)
    return false;

  if (a.y + a.h <= b.y + 0.1 )
    return false;

  if (a.y + 0.1 >= b.y+b.h)
    return false;



  return true;
}

svlObject2dFrame growToProportion(const svlObject2dFrame & input, double wOverh)
{
  svlObject2dFrame rval = input;
  
  for (unsigned i = 0; i < rval.size(); i++)
    {
      double p = rval[i].w / rval[i].h;
      double centerX = rval[i].x + 0.5 * rval[i].w;
      double centerY = rval[i].y + 0.5 * rval[i].h;


      if (p < wOverh)
	rval[i].w = wOverh * rval[i].h;
      else if (p > wOverh)
	rval[i].h =  rval[i].w / wOverh;

      rval[i].x = centerX - 0.5 * rval[i].w;
      rval[i].y = centerY - 0.5 * rval[i].h;

    }
  return rval;
}

svlObject2dFrame rects2objects(const vector<CvRect> & rects)
{
  svlObject2dFrame rval;
  for (unsigned i = 0; i < rects.size(); i++)
    rval.push_back(svlObject2d(rects[i].x,rects[i].y,rects[i].width,rects[i].height));
  return rval;
}

void scaleImages(vector<IplImage *> & v,int targW,int targH)
{
  for (unsigned i = 0; i < v.size(); i++)
    {
      //cout << "ratio: " << double(v[i]->width) / double(v[i]->height) << endl;
      resizeInPlace(&v[i],targH,targW,CV_INTER_LINEAR);
    }
}
