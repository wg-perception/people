/*********************************************************************
 * People-specific computer vision algorithms.
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

#include "face_detector/people.h"
#include <cfloat>

#define __PEOPLE_DEBUG__ 0
#define __PEOPLE_DISPLAY__ 0


People::People():
  list_(NULL),
  cv_image_gray_(0),
  disparity_image_(0),
  cam_model_(NULL),
  cft_r_plane_(NULL),
  cft_g_plane_(NULL),
  cft_b_plane_(NULL),
  cft_r_plane_norm_(NULL),
  cft_g_plane_norm_(NULL),
  cft_b_plane_norm_(NULL),
  rbins_(0),
  gbins_(0),
  cft_X_(NULL),
  cft_Y_(NULL),
  cft_Z_(NULL), 
  cft_start_hist_(NULL),
  cft_ratio_hist_(NULL){

#if __PEOPLE_DISPLAY__
  cvNamedWindow("Real face hist",CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Current face hist", CV_WINDOW_AUTOSIZE);
#endif

}

People::~People() {
  // Kill all the threads.
  threads_.interrupt_all();
  threads_.join_all();
  for (uint i=0; i<face_go_mutices_.size(); i++) {
    delete face_go_mutices_[i];
    face_go_mutices_[i] = NULL;
  }

  for (uint i=0; i < list_.size(); i++) {
    freePerson(i);
  }
  for (int i=list_.size(); i>0; i--) {
    list_.pop_back();
  }

  if (cv_image_gray_) {cvReleaseImage(&cv_image_gray_); cv_image_gray_ = 0; }
  //if (disparity_image_) {cvReleaseImage(&disparity_image_); disparity_image_ = 0;}
  disparity_image_ = 0;
  cam_model_ = 0;
  for (uint i=0; i<storages_.size(); i++) {
    if (storages_[i]) {cvReleaseMemStorage(&storages_[i]); storages_[i] = 0; }
    if (cascades_[i]) {cvReleaseHaarClassifierCascade(&cascades_[i]); cascades_[i] = 0; }
  }

  cvReleaseImage(&cft_r_plane_); cft_r_plane_ = 0; 
  cvReleaseImage(&cft_g_plane_); cft_g_plane_ = 0;
  cvReleaseImage(&cft_b_plane_); cft_b_plane_ = 0;
  cvReleaseImage(&cft_r_plane_norm_); cft_r_plane_norm_ = 0;
  cvReleaseImage(&cft_g_plane_norm_); cft_g_plane_norm_ = 0; 
  cvReleaseImage(&cft_b_plane_norm_); cft_b_plane_norm_ = 0; 
  cvReleaseImage(&cft_X_); cft_X_ = 0;
  cvReleaseImage(&cft_Y_); cft_Y_ = 0;
  cvReleaseImage(&cft_Z_); cft_Z_ = 0;
  cvReleaseMat(&rbins_); rbins_ = 0;
  cvReleaseMat(&gbins_); gbins_ = 0;
  cvReleaseHist(&cft_start_hist_); cft_start_hist_ = 0;
  cvReleaseHist(&cft_ratio_hist_); cft_ratio_hist_ = 0;

#if __PEOPLE_DISPLAY__
  cvDestroyWindow("Real face hist");
  cvDestroyWindow("Current face hist");
#endif

  }


void People::freePerson(int iperson) {
  if (list_[iperson].face_color_hist) {cvReleaseHist(&list_[iperson].face_color_hist);  list_[iperson].face_color_hist = 0;}
  if (list_[iperson].shirt_color_hist) {cvReleaseHist(&list_[iperson].shirt_color_hist); list_[iperson].shirt_color_hist = 0;}
  if (list_[iperson].body_mask_2d) {cvReleaseImage(&list_[iperson].body_mask_2d);    list_[iperson].body_mask_2d = 0;}
  if (list_[iperson].face_mask_2d) {cvReleaseImage(&list_[iperson].face_mask_2d);    list_[iperson].face_mask_2d = 0;}
}

/**************** HELPER FCNS *************************************/

// Computes the histogram of values in the image weighted by an Epanechnikov kernel.
static void calc_weighted_rg_hist_with_depth_2(IplImage* imager, IplImage* imageg, CvMat* center, IplImage *X, IplImage *Y, IplImage *Z, double band, CvHistogram* hist){
  CvRect bbox = cvGetImageROI(imager);
  cvClearHist(hist);

  int rbin,gbin,bbv1,bbv2,bbu1,bbu2;
  double d, dx, dy, dz, cx, cy, cz;
  float *xptr,*yptr,*zptr;
  cx = cvmGet(center,0,0);
  cy = cvmGet(center,0,1);
  cz = cvmGet(center,0,2);

  // Set the entire histogram to a small number to avoid 0 bins (and division by zero later).
  cvSet(hist->bins,cvScalar(0.0000001));//0.0000001)); //band*band*DBL_MIN
  band = band*band;
  bbv1 = bbox.y;
  bbv2 = (bbox.y+bbox.height);
  bbu1 =  bbox.x;
  bbu2 = (bbox.x+bbox.width);
  for (int v = bbv1; v<bbv2; v++) {
    xptr = (float *)(X->imageData + v*X->widthStep) + bbox.x;
    yptr = (float *)(Y->imageData + v*Y->widthStep) + bbox.x;
    zptr = (float *)(Z->imageData + v*Z->widthStep) + bbox.x;
    for (int u = bbu1; u<bbu2; u++) {
      if (*zptr != 0.0) {
	dz = ((*zptr) - cz); 
	dx = ((*xptr) - cx); xptr++;
	dy = ((*yptr) - cy); yptr++;
	d = (dx*dx+dy*dy+dz*dz)/band;
	rbin = floor(cvGetReal2D(imager,v-bbox.y,u-bbox.x)/2.0);
	//cvSetReal2D(imager,v-bbox.y,u-bbox.x,255);
	gbin = floor(cvGetReal2D(imageg,v-bbox.y,u-bbox.x)/2.0);
	//cvSetReal2D(imageg,v-bbox.y,u-bbox.x,255);
	if (d <= 1.0){// && 2 < rbin && rbin < 126 && 2 < gbin && gbin < 126) {
	  //rbin = cvGetReal2D(imager,v-bbox.y,u-bbox.x);
	  //rbin = (1.5*rbin <= 127) ? (int)1.5*rbin : 127;
	  //gbin = (int)0.75*gbin;

	  //rbin = floor(cvGetReal2D(imager,v-bbox.y,u-bbox.x)/2.0);
	  //gbin = floor(cvGetReal2D(imageg,v-bbox.y,u-bbox.x)/2.0);
	  cvSetReal2D(hist->bins, rbin, gbin, 1-d + cvGetReal2D(hist->bins,rbin,gbin)); 
	  // Ignore normalization constant since we'll normalize the histogram below.
	}
      }
      else {
	//cvSetReal2D(imager,v-bbox.y,u-bbox.x,0);
	//cvSetReal2D(imageg,v-bbox.y,u-bbox.x,0);	
      }
    }
  }

  cvNormalizeHist(hist,1.0);
  
}


/*****************************************************************/


// Set the tracking filter update time for a given person.
void People::setTrackingFilterUpdateTime(ros::Time time, int iperson) {
  list_[iperson].last_tracking_filter_update_time = time;
} 

// Return true if the time lapse between the last filter update and the
// given time is less than a set threshold.
bool People::isWithinTrackingFilterUpdateTimeThresh(ros::Time time, int iperson){
  return (time - list_[iperson].last_tracking_filter_update_time <= ros::Duration().fromSec(TRACKING_FILTER_TIMEOUT_S));
}

void People::killIfFilterUpdateTimeout(ros::Time time){
  for (uint iperson=0; iperson<list_.size(); iperson++) {
    if ((time - list_[iperson].last_tracking_filter_update_time) > ros::Duration().fromSec(TRACKING_FILTER_TIMEOUT_S)) {
      freePerson(iperson);
      list_.erase(list_.begin()+iperson);
      iperson--;
    }
  }
}

// Set a person's id
void People::setID(std::string id, int iperson){
  list_[iperson].id = id;
}

// Get a person's id
std::string People::getID(int iperson){
  return list_[iperson].id;
}

// Get the index of a person with a given id.
int People::findID(std::string id) {
  for (uint i=0; i<list_.size(); i++) {
    if (list_[i].id == id) {
      return i;
    }
  }
  return -1;
}

// Return the face size in 3D
double People::getFaceSize3D(int iperson) {
  return list_[iperson].face_size_3d;
}

// Set a person's 3D face size.
void People::setFaceSize3D(double face_size, int iperson) {
  if (face_size <= 0.0) {
    list_[iperson].face_size_3d = FACE_SIZE_DEFAULT_M;
  }
  else {
    list_[iperson].face_size_3d = face_size;
  }
}


// Takes a rectangle center and size and outputs the four corners in the row order TLxy, TRxy, BLxy, BRxy
// Assumes centers are one point per row.
// Doesn't do any bounds checking.
void People::centerSizeToFourCorners( CvMat *centers, CvMat *sizes, CvMat *four_corners) {
  CvSize csize = cvGetSize(centers);

  cvmSet(four_corners,0,0, cvmGet(centers,0,0)-cvmGet(sizes,0,0)); // TL
  cvmSet(four_corners,0,1, cvmGet(centers,0,1)-cvmGet(sizes,0,1)); // TL
  cvmSet(four_corners,1,0, cvmGet(centers,0,0)+cvmGet(sizes,0,0)); // TR
  cvmSet(four_corners,1,1, cvmGet(four_corners,0,1)); // TR
  cvmSet(four_corners,2,0, cvmGet(four_corners,0,0)); // BL
  cvmSet(four_corners,2,1, cvmGet(centers,0,1)+cvmGet(sizes,0,1)); // BL
  cvmSet(four_corners,3,0, cvmGet(four_corners,1,0)); // BR
  cvmSet(four_corners,3,1, cvmGet(four_corners,2,1)); // BR
  
#if __PEOPLE_DEBUG__
  printf("Center %f %f, size %f %f, opposite corners (%f, %f), (%f, %f)\n", 
	 cvGetReal2D(centers,0,0),cvGetReal2D(centers,0,1),
	 cvGetReal2D(sizes,0,0),cvGetReal2D(sizes,0,1),
	 cvmGet(four_corners,0,0), cvmGet(four_corners,0,1), cvmGet(four_corners,3,0), cvmGet(four_corners,3,1));
#endif

}

// Return the number of people.
int People::getNumPeople(){
  return list_.size();
}

// Add an empty person to the list of people.
void People::addPerson(){
  Person p;  
  p.face_color_hist = 0;
  p.shirt_color_hist = 0;
  p.body_mask_2d = 0;
  p.face_mask_2d = 0;
  p.id = "";
  list_.push_back(p);
}

// Set a person's face.
void People::setFaceBbox2D(CvRect face_rect, int iperson ) {
  list_[iperson].face_bbox_2d = face_rect; 
}

// Set a person's position.
void People::setFaceCenter3D(double cx, double cy, double cz, int iperson) {
  list_[iperson].face_center_3d = cvScalar(cx,cy,cz);
}

// Print a person's position.
void People::printFaceCenter3D(int iperson) {
  std::cout << "Face center for person " << iperson << ": " << list_[iperson].face_center_3d.val[0] << " " << list_[iperson].face_center_3d.val[1] << " " << list_[iperson].face_center_3d.val[2] << std::endl;
}


// Find the closest person with a face within a certain distance in 3D.
int People::findPersonFaceLTDist3D(double dist, double cx, double cy, double cz){
  double mindist = dist;
  int person = -1;
  for (uint iperson = 0; iperson < list_.size(); iperson++) {
    double tdist = pow(cx-list_[iperson].face_center_3d.val[0],2.0)+pow(cy-list_[iperson].face_center_3d.val[1],2.0) + pow(cz-list_[iperson].face_center_3d.val[2],2.0);
    if (tdist <= mindist) {
      mindist = dist;
      person = iperson;
    }
  }
  return person;    
}


// Make a histogram into an image. 
// out_im should have depth 8U, all values will be put between 0-255.
IplImage* People::faceHist2Im(int iperson){
  CvSize hsize = cvGetSize(list_[iperson].face_color_hist->bins);
  IplImage *out_im = cvCreateImage(hsize, IPL_DEPTH_8U, 1);
  uchar *cptr;
  for (int g=0; g<hsize.height; g++) {
    cptr = (uchar*)(out_im->imageData + g*out_im->widthStep);
    for (int r=0; r<hsize.width; r++) {
      (*cptr) = (uchar)(round(255.0*(cvQueryHistValue_2D(list_[iperson].face_color_hist,r,g))));
      cptr++; 
    }
  }
  return out_im;
}

// Clear a person's face histogram.
void People::clearFaceColorHist(int iperson) {
  cvReleaseHist(&list_[iperson].face_color_hist);
  list_[iperson].face_color_hist = 0;
}


/********
 * Detect all faces in an image.
 * Input:
 * image - The image in which to detect faces.
 * haar_classifier_filename - Path to the xml file containing the trained haar classifier cascade.
 * threshold - Detection threshold. Currently unused.
 * disparity_image - Image of disparities (from stereo). To avoid using depth information, set this to NULL.
 * cam_model - The camera model created by CvStereoCamModel.
 * do_draw - If true, draw a box on `image' around each face.
 * Output:
 * A vector of CvRects containing the bounding boxes around found faces.
 *********/ 



void People::initFaceDetection(uint num_cascades, std::vector<string> haar_classifier_filenames) {
  images_ready_ = 0;

  face_go_mutices_.resize(num_cascades);

  for (uint i=0; i<num_cascades; i++) {

    face_go_mutices_[i] = new boost::mutex();

    if (cascades_.size()<num_cascades) { 
      cascades_.push_back((CvHaarClassifierCascade*)cvLoad(haar_classifier_filenames[i].c_str()));
    }
    else if (!cascades_[i]) {
      cascades_[i] = (CvHaarClassifierCascade*)cvLoad(haar_classifier_filenames[i].c_str());
    }
    if (!cascades_[i]) {
      std::cerr << "Cascade file " << haar_classifier_filenames[i] << " doesn't exist.\n" << std::endl;
      return;
    }
    if (storages_.size()<num_cascades) {
      storages_.push_back(cvCreateMemStorage(0));
    }
    else if (!storages_[i]) {
      storages_[i] = cvCreateMemStorage(0);
    }
    threads_.create_thread(boost::bind(&People::faceDetectionThread,this,i));
  }
}

/////

vector<Box2D3D> People::detectAllFaces(IplImage *image, double threshold, IplImage *disparity_image, CvStereoCamModel *cam_model) {

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
  for (uint it=0; it<face_go_mutices_.size(); it++) {
    boost::mutex::scoped_lock fgmlock(*(face_go_mutices_[it]));
    images_ready_++;
    fgmlock.unlock();
  }

  face_detection_ready_cond_.notify_all();

  boost::mutex::scoped_lock fdmlock(face_done_mutex_);
  while (num_threads_to_wait_for_ > 0) {
    face_detection_done_cond_.wait(fdmlock);
  }  

  return faces_;
}

/////

void People::faceDetectionThread(uint i) {

  while (1) {
    boost::mutex::scoped_lock fgmlock(*(face_go_mutices_[i]));
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
    int this_min_face_size = (int)(floor(cam_model_->getDeltaU(FACE_SIZE_MIN_M, MAX_Z_M)));

    CvSeq *face_seq = cvHaarDetectObjects(cv_image_gray_, cascades_[i], storages_[i], 1.2, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(this_min_face_size,this_min_face_size));

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

      if (disparity_image_ && cam_model_) {
    
	// Get the median disparity in the middle half of the bounding box.
	avg_disp = cvMedianNonZeroElIn2DArr(disparity_image_,
					    floor(one_face.box2d.y+0.25*one_face.box2d.height),floor(one_face.box2d.y+0.75*one_face.box2d.height),
					    floor(one_face.box2d.x+0.25*one_face.box2d.width), floor(one_face.box2d.x+0.75*one_face.box2d.width)); 

	one_face.center2d = cvScalar(one_face.box2d.x+one_face.box2d.width/2.0,
				     one_face.box2d.y+one_face.box2d.height/2.0,
				     avg_disp);
	one_face.radius2d = one_face.box2d.width/2.0;

	if (avg_disp > 0) {
	  one_face.radius3d = cam_model_->getDeltaX(one_face.box2d.width,avg_disp)/2.0;
	  cvmSet(uvd,0,0,one_face.center2d.val[0]);
	  cvmSet(uvd,0,1,one_face.center2d.val[1]);
	  cvmSet(uvd,0,2,avg_disp);
	  cam_model_->dispToCart(uvd,xyz);      
	  one_face.center3d = cvScalar(cvmGet(xyz,0,0),cvmGet(xyz,0,1),cvmGet(xyz,0,2));
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


/*********************************************
 * Perform one frame of color-based tracking of the face blob.
 * Uses mean shift-based tracking on the Bhattacharya coeff of the colour histograms in the RG (normalized) colourspace.
 * Reference: Comaniciu and Ramesh, Robust Detection and Tracking of Human Faces with an Active Camera, IEEE Visual Surveillance 2000
 * Input:
 *   image: IplImage in color.
 *   disparity_image: IplImage of the disparity values.
 *   cam_model: The stereo camera model.
 *   npeople: The number of people to track.
 *   which_people: Indices of the people to track.
 *   start_points: Starting position for each person. If the list is empty, start at the person's current position.
 * Output:
 *   end_points: List of the new face centers. Does *not* update the face centers of the people without further intervention. Assumes the endpoint array is allocated.
 *   Side effect: can update the person's colour histogram.
 *********************************************/
bool People::track_color_3d_bhattacharya(const IplImage *image, IplImage *disparity_image, CvStereoCamModel *cam_model, double kernel_radius_m, int npeople, int* t_which_people, CvMat* start_points, CvMat* end_points, bool* tracked_each) {
 
#if __PEOPLE_DEBUG__
  printf("in tracker\n");
#endif

  // If the image isn't the right number of colors (ie it's grayscale), just return.
  if (image->nChannels != 3) {
    return false;
  }

  CvSize imsize = cvGetSize(image);

  // By default: track everyone.
  int *which_people;
  if (npeople==0) {
    npeople = list_.size();
    which_people = new int[npeople];
    for (int i=0; i<npeople; i++) {
      which_people[i] = i;
    }
  }
  else {
    which_people = new int[npeople];
    memcpy(which_people, t_which_people, npeople*sizeof(int));
  }

  // If the starting positions weren't given, init them to the face positions.
  if (!start_points) {
    start_points = cvCreateMat(npeople,3,CV_32FC1);
    for (int iperson = 0; iperson<npeople; iperson++) {
      for (int i=0; i<3; i++) {
	cvmSet(start_points,iperson,i, list_[which_people[iperson]].face_center_3d.val[i]);
      }
    }
  }
  
  // The mean shift bandwidths will be the same as the face sizes. Can change this later.
  int hsizes[] = {128,128};
  // For the colour conversion and histogram...
  float range[] = {0,255};
  float* ranges[] = {range, range};


  // Check that the memory allocated to temp images is of the right size. If not, release and reallocate it.
  bool alloc = false;
  if (!cft_r_plane_) {
    alloc = true;
  }
  else {
    CvSize old_imsize = cvGetSize(cft_r_plane_);
    if (imsize.width!=old_imsize.width || imsize.height!=old_imsize.height) {

      alloc = true;

      cvReleaseImage(&cft_r_plane_); cft_r_plane_ = 0; 
      cvReleaseImage(&cft_g_plane_); cft_g_plane_ = 0;
      cvReleaseImage(&cft_b_plane_); cft_b_plane_ = 0;
      cvReleaseImage(&cft_r_plane_norm_); cft_r_plane_norm_ = 0; 
      cvReleaseImage(&cft_g_plane_norm_); cft_g_plane_norm_ = 0;
      cvReleaseImage(&cft_b_plane_norm_); cft_b_plane_norm_ = 0;
      cvReleaseImage(&cft_X_); cft_X_ = 0;
      cvReleaseImage(&cft_Y_); cft_Y_ = 0;
      cvReleaseImage(&cft_Z_); cft_Z_ = 0; 
      cvReleaseMat(&rbins_); rbins_ = 0;
      cvReleaseMat(&gbins_); gbins_ = 0;
      cvReleaseMat(&cft_uvd_); cft_uvd_ = 0;
      cvReleaseMat(&cft_xyz_); cft_xyz_ = 0;
    }
  }
  if (alloc) {
    cft_r_plane_ = cvCreateImage( imsize, IPL_DEPTH_8U, 1);
    cft_g_plane_ = cvCreateImage( imsize, IPL_DEPTH_8U, 1);
    cft_b_plane_ = cvCreateImage( imsize, IPL_DEPTH_8U, 1);
    cft_r_plane_norm_ = cvCreateImage( imsize, IPL_DEPTH_8U, 1);
    cft_g_plane_norm_ = cvCreateImage( imsize, IPL_DEPTH_8U, 1);
    cft_b_plane_norm_ = cvCreateImage( imsize, IPL_DEPTH_8U, 1);
    cft_X_ = cvCreateImage( imsize, IPL_DEPTH_32F, 1 );
    cft_Y_ = cvCreateImage( imsize, IPL_DEPTH_32F, 1 );
    cft_Z_ = cvCreateImage( imsize, IPL_DEPTH_32F, 1 );
    rbins_ = cvCreateMat( imsize.height, imsize.width, CV_8UC1);
    gbins_ = cvCreateMat( imsize.height, imsize.width, CV_8UC1);
    cft_uvd_ = cvCreateMat(imsize.width*imsize.height,3,CV_32FC1);
    cft_xyz_ = cvCreateMat(imsize.width*imsize.height,3,CV_32FC1);
    cft_start_hist_ = cvCreateHist(2, hsizes, CV_HIST_ARRAY, ranges, 1);
    cft_ratio_hist_ = cvCreateHist(2, hsizes, CV_HIST_ARRAY, ranges, 1);
  }
  else {
    cvClearHist(cft_start_hist_);
    cvClearHist(cft_ratio_hist_);
  }


  // Convert the image to the right colourspace.
  // Nothing needs to be done b/c we're going to use the R and G components and normalize the intensity.
  //cvCvtPixToPlane( image, cft_b_plane_, cft_g_plane_, cft_r_plane_, 0);
  cvCvtPixToPlane( image, cft_b_plane_norm_, cft_g_plane_norm_, cft_r_plane_norm_, 0);
  //cvEqualizeHist( cft_r_plane_, cft_r_plane_norm_);
  //cvEqualizeHist( cft_g_plane_, cft_g_plane_norm_);


  // Get the 3d coords for each point.
  
  //cvDilate(disparity_image, disparity_image);
  //cvErode(disparity_image, disparity_image);

  float *fptr = (float*)(cft_uvd_->data.ptr);
  ushort *cptr;// = (uchar*)(disparity_image->imageData);
  for (int v =0; v < imsize.height; v++) {
    cptr = (ushort*)(disparity_image->imageData+v*disparity_image->widthStep);
    for (int u=0; u<imsize.width; u++) {
      (*fptr) = (float)u; fptr++;
      (*fptr) = (float)v; fptr++;
      (*fptr) = (float)(*cptr); fptr++; cptr++;      
    }
  }
  cam_model->dispToCart(cft_uvd_,cft_xyz_);
  fptr = (float*)(cft_xyz_->data.ptr);
  float *fptr1;// = (float*)(cft_X_->imageData);
  float *fptr2;// = (float*)(cft_Y_->imageData);
  float *fptr3;// = (float*)(cft_Z_->imageData);
  for (int v =0; v < imsize.height; v++) {
    fptr1 = (float*)(cft_X_->imageData+v*cft_X_->widthStep);
    fptr2 = (float*)(cft_Y_->imageData+v*cft_Y_->widthStep);
    fptr3 = (float*)(cft_Z_->imageData+v*cft_Z_->widthStep);
    for (int u=0; u<imsize.width; u++) {
      (*fptr1) = (*fptr); fptr1++; fptr++;
      (*fptr2) = (*fptr); fptr2++; fptr++;
      (*fptr3) = (*fptr); fptr3++; fptr++;
    }
  }
 
  /*** Mean shift tracking of the face centers in the normalized RG colourspace using the Bhattacharya coeff of the colour histograms. ***/

  CvRect bbox;
  double d, dx, dy, dz;
  CvMat* four_corners = cvCreateMat(4,3,CV_32FC1);
  CvMat* four_corners_2d = cvCreateMat(4,3,CV_32FC1);
  CvMat* size_3d = cvCreateMat(1,3,CV_32FC1);
  CvMat* my_start_point = cvCreateMatHeader(1,3,CV_32FC1);

  CvMat* curr_point = NULL;// = cvCreateMatHeader(1,3,CV_32FC1);
  CvMat* next_point = cvCreateMat(1,3,CV_32FC1);

  CvMat* tpt = cvCreateMat(1,3,CV_32FC1);
  Person *t_person;

#if __PEOPLE_DISPLAY__
  CvSize histsize = cvSize(hsizes[0],hsizes[1]);
  IplImage *hist_im = cvCreateImage(histsize, IPL_DEPTH_8U, 1);
  IplImage *hist_im_real = cvCreateImage(histsize,IPL_DEPTH_8U, 1);

#endif

  for (int iperson = 0; iperson<npeople; iperson++) {
    t_person = &list_[which_people[iperson]];

    tracked_each[iperson] = true;

    cvSet(size_3d, cvScalar(t_person->face_size_3d));

    /*** Compute the histogram at the current position.***/
    cvClearHist(cft_start_hist_);

    // Convert 3d point-size to 2d rectangle
    cvGetRow(start_points,my_start_point,iperson);

#if __PEOPLE_DEBUG__
    printf("My start point: ");
    for (int ip = 0; ip < 3; ip++) {
      printf("%f ", cvmGet(my_start_point,0,ip));
    }
    printf("\n"); 
#endif

    centerSizeToFourCorners(my_start_point, size_3d, four_corners);
    for (int ic=0; ic<4; ic++) {
      cvmSet(four_corners, ic, 2, cvmGet(my_start_point,0,2));
    }

    cam_model->cartToDisp(four_corners, four_corners_2d);

    if (cvmGet(four_corners_2d,0,0)< 0 || cvmGet(four_corners_2d,0,1) < 0 || cvmGet(four_corners_2d,1,0) >= imsize.width || cvmGet(four_corners_2d,2,1) >= imsize.height) {
      cvmSet(end_points,iperson,0,cvmGet(my_start_point,0,0));
      cvmSet(end_points,iperson,1,cvmGet(my_start_point,0,1));
      cvmSet(end_points,iperson,2,cvmGet(my_start_point,0,2));
      tracked_each[iperson] = false;
      //printf("bbox1 bad\n");
      //printf("x1,y1,x2,y2,w,h %f %f %f %f %d %d \n",cvmGet(four_corners_2d,0,0), cvmGet(four_corners_2d,0,1),cvmGet(four_corners_2d,1,0),cvmGet(four_corners_2d,2,1),imsize.width, imsize.height);
      continue;
    }

    bbox = cvRect((int)(cvmGet(four_corners_2d,0,0)), 
		  (int)(cvmGet(four_corners_2d,0,1)), 
		  (int)(cvmGet(four_corners_2d,1,0)-cvmGet(four_corners_2d,0,0)+1), 
		  (int)(cvmGet(four_corners_2d,2,1)-cvmGet(four_corners_2d,0,1)+1));
  
    // Calc histogram
    cvSetImageROI(cft_r_plane_norm_,bbox);
    cvSetImageROI(cft_g_plane_norm_,bbox);
    calc_weighted_rg_hist_with_depth_2(cft_r_plane_norm_, cft_g_plane_norm_, my_start_point, cft_X_, cft_Y_, cft_Z_, t_person->face_size_3d, cft_start_hist_);
    cvResetImageROI(cft_r_plane_norm_);
    cvResetImageROI(cft_g_plane_norm_);

    /*** If this is the first frame for this person, set their histogram and return the start point as the new point (don't track). ***/
    if (!t_person->face_color_hist) {
      cvCopyHist(cft_start_hist_, &t_person->face_color_hist);
      fptr1 = (float*)(start_points->data.ptr + iperson*start_points->step);
      fptr2 = (float*)(end_points->data.ptr + iperson*end_points->step); 
      (*fptr2) = (*fptr1); fptr1++; fptr2++;
      (*fptr2) = (*fptr1); fptr1++; fptr2++;
      (*fptr2) = (*fptr1); fptr1 = NULL; fptr2 = NULL;
      continue;
    }


    /*** Mean shift iterations. ***/
    double EPS = COLOR_TRACK_EPS_M*COLOR_TRACK_EPS_M; // Squared!!!
    curr_point = cvCloneMat(my_start_point);
    cvSetZero(next_point);
    double bhat_coeff, bhat_coeff_new;
    double total_weight = 0.0;
    double wi = 0.0;
    int r_bin, g_bin;
    float cp[3];
    int u1, u2, v1, v2;
    float *xptr, *yptr, *zptr, *nptr;
    uchar *rptr, *gptr;
    double kernel_mult = kernel_radius_m / t_person->face_size_3d;//4.0; // Magical kernel size multiplier. Kernel size is kernel_mult * t_person->face_size_3d. (3d face size is the radius of the face, *not* the diameter.)
    double denom;

    // Compute the Bhattacharya coeff of the start hist vs the true face hist.
    bhat_coeff = cvCompareHist( t_person->face_color_hist, cft_start_hist_, CV_COMP_BHATTACHARYYA);
    int iter;
    for (iter = 0; iter < COLOR_TRACK_MAX_ITERS; iter++) {

#if __PEOPLE_DEBUG__
      printf("iter %d\n",iter);
#endif

      // Compute the next location of the center
      d = 0.0;
      total_weight = 0.0;
      wi = 0.0;
      cvSetZero(next_point);
      fptr1 = (float*)(curr_point->data.ptr);
      cp[0] = (*fptr1); fptr1++;
      cp[1] = (*fptr1); fptr1++;
      cp[2] = (*fptr1); fptr1++;
      //printf("try div\n\n");
      // cvDiv(&(t_person->face_color_hist->mat), &(start_hist->mat), &(ratio_hist->mat));
      //cvPow(&(ratio_hist->mat),&(ratio_hist->mat),0.5);
      //printf("done div\n\n");
      

      // For each pixel within the kernel distance of the current position...
      if (cp[2]==0.0) {
	// The current point should never have a z-val of 0. 
	printf("Z-val for the current pt (x,y) (%f,%f) on iter %d is 0\n",cvmGet(curr_point,0,0), cvmGet(curr_point,0,1), iter);
	break;
      }

      // Converting the scale here controls the size of the kernel we'll use.
      
      cvConvertScale(size_3d,size_3d,kernel_mult,0);
      centerSizeToFourCorners(curr_point, size_3d, four_corners);
      cvConvertScale(size_3d,size_3d,1./kernel_mult,0);
      cam_model->cartToDisp(four_corners, four_corners_2d);   
      
      u1 = MAX((int)cvmGet(four_corners_2d,0,0),0);
      u2 = MIN((int)cvmGet(four_corners_2d,1,0),cft_X_->width);
      v1 = MAX((int)cvmGet(four_corners_2d,0,1),0);
      v2 = MIN((int)cvmGet(four_corners_2d,2,1),cft_X_->height);
      //bbox = cvRect(u1, v1, u2-u1+1, v2-v1+1);
      denom = kernel_mult*t_person->face_size_3d;
      denom *= denom;
#if __PEOPLE_DEBUG__
      printf("ds ");
#endif
      int total_z=0;
      int total_close=0;
      int total_inbounds = 0;
      for (int v=v1; v<=v2; v++) {
	xptr = (float*)(cft_X_->imageData + v*cft_X_->widthStep) + u1;
	yptr = (float*)(cft_Y_->imageData + v*cft_Y_->widthStep) + u1;
	zptr = (float*)(cft_Z_->imageData + v*cft_Z_->widthStep) + u1;
	rptr = (uchar*)(cft_r_plane_norm_->imageData + v*cft_r_plane_norm_->widthStep) + u1;
	gptr = (uchar*)(cft_g_plane_norm_->imageData + v*cft_g_plane_norm_->widthStep) + u1;
#if __PEOPLE_DEBUG__
	printf("\n");
#endif
	for (int u = u1; u<=u2; u++) {
	  if ((*zptr) != 0.0) {

	    total_z++;

	    dx = (*xptr)-cp[0];
	    dy = (*yptr)-cp[1];
	    dz = (*zptr)-cp[2];
	    d = (dx*dx + dy*dy + dz*dz)/denom;
#if __PEOPLE_DEBUG__
	    printf("%f %f %f ",*zptr,cp[2],dz);
#endif
	    r_bin = (int)(floor((float)(*rptr)/2.0));
	    g_bin = (int)(floor((float)(*gptr)/2.0));
	    if (d<=1.0) total_close++;
	    if (d <= 1.0){// && 2 < r_bin && r_bin < 126 && 2 < g_bin && g_bin < 126) {
	      total_inbounds++;
	      //r_bin = (1.5*r_bin <= 127) ? (int)1.5*r_bin : 127;
	      //g_bin = (int)0.75*g_bin;

	      wi = sqrt(cvQueryHistValue_2D(t_person->face_color_hist,r_bin,g_bin)/cvQueryHistValue_2D(cft_start_hist_,r_bin,g_bin));
	      //printf("rbin %d gbin %d facehist %f starthist %f sqrt %f\n",r_bin,g_bin,cvQueryHistValue_2D(t_person->face_color_hist,r_bin,g_bin),cvQueryHistValue_2D(cft_start_hist_,r_bin,g_bin), wi);
       	      total_weight += wi;
	      nptr = (float *)(next_point->data.ptr);
	      (*nptr) += wi*dx; nptr++;
	      (*nptr) += wi*dy; nptr++;
	      (*nptr) += wi*dz;
	    }
	  }

	  xptr++; yptr++; zptr++;
	  rptr++; gptr++;
	}
      }

      //printf("\ntotal z %d, total_close %d,  total inbounds %d\n", total_z, total_close, total_inbounds);
#if __PEOPLE_DEBUG__
      printf("\n\n\n");
#endif

      // If no points in this kernel, don't move the point.
      if (total_weight == 0.0) {
	printf("No points in kernel\n");
	printf("currpoint x %f y %f z %f\n",cvmGet(curr_point,0,0),cvmGet(curr_point,0,1),cvmGet(curr_point,0,2));
	break;
      }
      d = 0.0;      
      nptr = (float *)(next_point->data.ptr);
      for (int i=0; i<3; i++) {
	(*nptr) /= total_weight;
	d += (*nptr)*(*nptr);
	(*nptr) += cp[i];
	nptr++;
      } 

      bhat_coeff_new = 0.0;
      bool ok_adjust = true; 
      while (true) {
	// Compute the histogram and Bhattacharya coeff at the new position.
	// Convert 3d point-size to 2d rectangle
	centerSizeToFourCorners(next_point, size_3d, four_corners);
	cam_model->cartToDisp(four_corners, four_corners_2d);

	if (cvmGet(four_corners_2d,0,0)< 0 || cvmGet(four_corners_2d,0,1) < 0 || cvmGet(four_corners_2d,1,0) >= imsize.width || cvmGet(four_corners_2d,2,1) >= imsize.height) {
	  //cvmSet(end_points,iperson,0,cvmGet(curr_point,0,0));cvmSet(end_points,iperson,1,cvmGet(curr_point,0,1));cvmSet(end_points,iperson,2,cvmGet(curr_point,0,2));
	  tracked_each[iperson] = false;
	  ok_adjust = false;
	  break;
	}

	bbox = cvRect(cvmGet(four_corners_2d,0,0), cvmGet(four_corners_2d,0,1), 
		      cvmGet(four_corners_2d,1,0)-cvmGet(four_corners_2d,0,0)+1, 
		      cvmGet(four_corners_2d,2,1)-cvmGet(four_corners_2d,0,1)+1);
	// Set roi to 2d rectangle and normalize r and g within the roi.      
	//cvSetImageROI( cft_r_plane_, bbox );
	//cvSetImageROI( cft_g_plane_, bbox );
        cvSetImageROI( cft_r_plane_norm_, bbox);
	cvSetImageROI( cft_g_plane_norm_, bbox);
	//cvEqualizeHist( cft_r_plane_, cft_r_plane_norm_);
	//cvEqualizeHist( cft_g_plane_, cft_g_plane_norm_);
	// Calc histogram
	calc_weighted_rg_hist_with_depth_2(cft_r_plane_norm_, cft_g_plane_norm_, next_point, cft_X_, cft_Y_, cft_Z_, t_person->face_size_3d, cft_start_hist_);
	// Reset ROIs
	//cvResetImageROI( cft_r_plane_);
	//cvResetImageROI( cft_g_plane_);
	cvResetImageROI( cft_r_plane_norm_);
	cvResetImageROI( cft_g_plane_norm_);
	bhat_coeff_new = cvCompareHist( t_person->face_color_hist, cft_start_hist_, CV_COMP_BHATTACHARYYA);  

	if ((d > EPS) && (bhat_coeff_new < bhat_coeff)) {
	  d = 0.0;
	  fptr = (float*)(next_point->data.ptr);
	  // Set the next point as the avg of the next point and current point.
	  for (int i=0; i<3; i++) {
	    (*fptr) += cp[i];
	    (*fptr) /= 2.0;
	    d += ((*fptr)-cp[i])*((*fptr)-cp[i]);
	    fptr++;
	  }
	}
	else {
	  break;
	}
      }

      cvCopy(next_point,curr_point);
      cvSetZero(next_point);
      bhat_coeff = bhat_coeff_new;
      
      // Movement is small, break.
      if (d <= EPS || !ok_adjust) {
	break;
      }


      // start_hist was already updated in the loop above.
      
    }

    // Set the new point.
    fptr1 = (float*)(curr_point->data.ptr);
    fptr2 = (float*)(end_points->data.ptr + iperson*end_points->step);
    for (int i=0; i<3; i++) { 
      (*fptr2) = (*fptr1); fptr2++; fptr1++;;
    }


#if __PEOPLE_DEBUG__
    printf("iter=%d, d=%f\n",iter, d);
#endif

#if __PEOPLE_DEBUG__
    printf("end point ");
    for (int i =0; i<3; i++) {
      printf("%f ",cvmGet(end_points,iperson,i));
    }
    printf("\n");

#endif


#if __PEOPLE_DISPLAY__
    uchar *cptr;
    float min, max;
    cvGetMinMaxHistValue( t_person->face_color_hist, &min, &max);

    for (int g=0; g<histsize.height; g++) {
      cptr = (uchar*)(hist_im_real->imageData + g*hist_im_real->widthStep);
      for (int r=0; r<histsize.width; r++) {
	(*cptr) = (uchar)(round(255.0*(cvQueryHistValue_2D(t_person->face_color_hist,r,g))/max));
	cptr++;
      }
    }

    cvGetMinMaxHistValue( t_person->face_color_hist, &min, &max);
    for (int g=0; g<histsize.height; g++) {
      cptr = (uchar*)(hist_im->imageData + g*hist_im->widthStep);
      for (int r=0; r<histsize.width; r++) {
	(*cptr) = (uchar)(MIN(255,round(255.0*(cvQueryHistValue_2D(cft_start_hist_,r,g))/max)));
	cptr++;
      }
    }
    cvShowImage("Real face hist",hist_im_real);
    cvShowImage("Current face hist",hist_im);

    //cvShowImage("Real face hist",cft_r_plane_norm_); //
    //cvShowImage("Current face hist",cft_g_plane_norm_); //
    //cvShowImage("blue",cft_b_plane_norm_); //
#endif

  }



  /*** Cleanup ***/ 
  cvReleaseMat(&four_corners); four_corners = 0;
  cvReleaseMat(&four_corners_2d); four_corners_2d = 0;
  cvReleaseMat(&size_3d); size_3d = 0;
  cvReleaseMat(&my_start_point); my_start_point = 0;
  cvReleaseMat(&next_point); next_point = 0;
  cvReleaseMat(&curr_point); curr_point = 0;
  cvReleaseMat(&tpt); tpt = 0;
#if __PEOPLE_DISPLAY__
  cvReleaseImage(&hist_im); hist_im = 0;
  cvReleaseImage(&hist_im_real); hist_im_real = 0;
#endif

  delete [] which_people;
  return true;
}



