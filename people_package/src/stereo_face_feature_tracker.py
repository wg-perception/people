import roslib
roslib.load_manifest('people')
import rospy
import rostest
import face_detection
from visualodometer import VisualOdometer, FeatureDetectorStar, DescriptorSchemeCalonder, DescriptorSchemeSAD
import camera
from std_msgs.msg import String
from deprecated_msgs.msg import ImageArray
from geometry_msgs.msg import PointStamped, PointCloud, Point
from people.msg import PositionMeasurement
import visual_odometry as VO
import starfeature
import calonder

from stereo import SparseStereoFrame

import sys
sys.path.append('lib')

import Image
import ImageChops
import ImageDraw

import random
import time
import math
import numpy

import os
import copy

DEBUG = True
SAVE_PICS = True
DESCRIPTOR = 'CALONDER'

################## DRAW_X #####################################################
def draw_x(draw, center, radius, color):
  draw.line((center[0]-radius[0], center[1]-radius[1], center[0]+radius[0], center[1]+radius[1]), fill=color)
  draw.line((center[0]-radius[0], center[1]+radius[1], center[0]+radius[0], center[1]-radius[1]), fill=color)


################## CLASS TRACKER CHILD OF CLASS VISUALODOMETER ################
class Tracker(VisualOdometer): #(visualodometer.VisualOdometer) :

  def collect_descriptors_sad(self, frame):
    self.timer['descriptor_collection'].start()
    frame.descriptors = [ VO.grab_16x16(frame.rawdata, frame.size[0], p[0]-7, p[1]-7) for p in frame.kp ]
    self.timer['descriptor_collection'].stop()


################## CLASS PEOPLETRACKER #############
class PeopleTracker:
  def __init__(self):
    self.usebag = False
    self.visualize = False
    self.keyframes = []
    self.current_keyframes = []
    self.p = people.people()
    self.cam = None
    self.camparams = None
    self.vo = None
    self.feature_detector = FeatureDetectorStar() #visualodometer.FeatureDetectorStar()
    self.feature_detector.thresh = 3.0
    self.cascade_file = "cascades/haarcascade_frontalface_alt.xml"
    assert os.access(self.cascade_file, os.R_OK)
    self.faces = None
    self.seq = 0
    self.keyframe_maxlength = 1
    self.desc_diff_thresh = -1.0
    self.num_feats = 40
    self.line_thresh = 10.0
    self.num_scales = 6
    self.feats_to_centers = []
    self.feats_to_centers_3d = []
    self.face_centers_3d = []
    self.real_face_sizes_3d = []
    self.min_real_face_size = 100 # in mm
    self.max_real_face_size = 600 # in mm
    self.good_points_thresh = 0.5
    self.max_face_dist = 3000 # in mm
    self.recent_good_frames = []
    self.recent_good_rects = []
    self.recent_good_centers_3d = []
    self.recent_good_motion = []
    self.recent_good_motion_3d = []
    self.same_key_rgfs = []
    self.pub = []


################### RECT_TO_CENTER_DIFF #######################
  def rect_to_center_diff(self, rect):
    diff = ((rect[2]-1)/2.0, (rect[3]-1)/2.0)
    center = (rect[0]+diff[0], rect[1]+diff[1])
    return center, diff

################### GET_FEATURES ##############################
  def get_features(self, frame, target_points, rect2d, censize3d):
    
    if DEBUG:
      print "Features from rect ", rect2d
    
    # Clear out all of the match info since it's not valid anymore.
    frame.matches = []
    frame.desc_diffs = []
    frame.good_matches = []

    # Extract 2d keypoints and get their disparities
    full = Image.fromstring("L",frame.size,frame.rawdata)
    (x,y,w,h) = (int(rect2d[0]), int(rect2d[1]), int(rect2d[2]), int(rect2d[3]))
    incr = 16
    subim = full.crop((x-incr,y-incr,x+w+incr,y+h+incr))
    
    sd = starfeature.star_detector(w+2*incr, h+2*incr, self.num_scales, self.feature_detector.thresh, self.line_thresh)
    results = [ (x1,y1) for (x1,y1,s1,r1) in sd.detect(subim.tostring()) ]
    frame.kp2d = [(x-incr+x1,y-incr+y1) for (x1,y1) in results if (incr<x1) and (incr<y1) and (x1<w+incr) and (y1<h+incr)]
    if not frame.kp2d:
      frame.kp2d = []
      frame.kp = []
      frame.avgd = -1.0
      print "No features"
      return

    self.vo.find_disparities(frame)

    # Convert to 3d keypoints and remove entries with invalid depths. I'm going to limit it to faces that are within a max dist as well.
    frame.kp = [kp for kp in frame.kp if 0.0<kp[2]]
    if not frame.kp:
      frame.kp2d = []
      frame.kp = []
      frame.avgd = -1.0
      print "no features"
      return
      
    frame.kp3d = [self.cam.pix2cam(kp[0],kp[1],kp[2]) for kp in frame.kp]
    to_remove = [kp3d[2]>self.max_face_dist or kp3d[2]<censize3d[2]-censize3d[3] or kp3d[2]>censize3d[2]+censize3d[3] for kp3d in frame.kp3d]
    # But only remove the entries if there are a few left. Otherwise, leave them all.
    if True: # sum(to_remove)<len(frame.kp)-2:
      #for k in [frame.kp, frame.kp3d]:
      frame.kp = [kp for (kp,remove) in zip(frame.kp,to_remove) if not remove]
      frame.kp3d = [kp for (kp,remove) in zip(frame.kp3d,to_remove) if not remove]
    if frame.kp:
      print len(frame.kp)
    else:
      print "0"
    # Enforce that kp2d and kp are the same. I would remove kp2d altogether except the stereo code uses it.
    frame.kp2d = [[kp[0],kp[1]] for kp in frame.kp]
    if frame.kp:
      frame.avgd = sum([kp[2] for kp in frame.kp])/len(frame.kp)
    else:
      frame.avgd = -1.0


################### GET_FEATURES ##############################
  def get_features_new(self, frame, target_points, rect, extra_incr):
    
    if DEBUG:
      print "Features from rect ", rect
    
    full = Image.fromstring("L",frame.size,frame.rawdata)
    (x,y,w,h) = (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
    innerbox = [max(0,x-extra_incr), max(0,y-extra_incr)]
    innerbox += [min(full.width, x+w+extra_incr), min(full.height, y+h+extra_incr)]
    incr = 16
    outerbox = [max(0,innerbox[0]-incr), max(0,innerbox[1]-incr)]
    outerbox += [min(full.width, innerbox[2]+incr), min(full.height, innerbox[3]+incr)] 
    subim = full.crop(outerbox)
    
    sd = starfeature.star_detector(outerbox[2]-outerbox[0]+1, outerbox[3]-outerbox[1]+1, self.num_scales, self.feature_detector.thresh, self.line_thresh)
    results = [ (x1,y1) for (x1,y1,s1,r1) in sd.detect(subim.tostring()) ]
    real_incr = [abs(i1-o1) for (i1,o1) in zip(innerbox,outerbox)]
    return [(x1-real_incr[0]+1,y1-real_incr[1]+1) for (x1,y1) in results if (real_incr[0]<x1) and (real_incr[1]<y1) and (x1<outerbox[2]-real_incr[2]) and (y1<outerbox[3]-real_incr[3])]

################### MAKE_FACE_MODEL ###########################
  def make_face_model(self, wincenter, maxdiff,  feat_list) :
    wc = numpy.array(wincenter)
    fl = numpy.array(feat_list)
    feats_to_center = wc-fl
    return feats_to_center.tolist()

#################### MEAN SHIFT ###############################
# start_point = Point from which to start tracking.
# sparse_pred_list = List of non-zero probabilities/predictions. 
#                    Each prob has the form: [x,y,...] 
# probs = A weight for each prediction
# bandwidths = The size of the (square) window around each prediction
# max_iter = The maximum number of iterations
# eps = The minimum displacement
  def mean_shift_sparse(self, start_point, sparse_pred_list, probs, bandwidths, max_iter, eps) :
    if len(sparse_pred_list)==0:
      return start_point
    total_weight = 0.0
    new_point = numpy.array(start_point)
    pred_list = numpy.array(sparse_pred_list)
    dpoint = numpy.array(0.0*pred_list[0])
    bands_sqr = [pow(b,2.0) for b in bandwidths]
    eps = eps*eps
    # For each iteration
    for iter in range(0,max_iter) :
      dpoint = 0.0*dpoint
      t_bands_sqr = copy.copy(bands_sqr)
      # Add each prediction in the kernel to the displacement
      ntries = 0
      while total_weight == 0.0:
        for ipred in range(len(pred_list)) :
          diff = pred_list[ipred]-new_point
          dist = numpy.dot(diff,diff)
          if dist < t_bands_sqr[ipred]: 
            total_weight += probs[ipred]
            dpoint += probs[ipred]*diff
          # If there weren't any predictions in the kernel, return the old point.
          if total_weight == 0.0 :
            if ntries < 5:
              t_bands_sqr = [4.0*tbs for tbs in t_bands_sqr]
            else:
              return new_point.tolist()
      # Otherwise, move the point
      dpoint /= total_weight
      new_point = new_point + dpoint
        
      # If the displacement was small, return the point
      if numpy.dot(dpoint,dpoint) <= eps :
        return new_point.tolist()

    # Reached the maximum number of iterations, return
    return new_point.tolist()


################### PARAMETER CALLBACK ########################
  def params(self, pmsg):

    if not self.vo:
      self.cam = camera.VidereCamera(pmsg.data)
      if DESCRIPTOR=='CALONDER':
        self.vo = VisualOdometer(self.cam, descriptor_scheme = DescriptorSchemeCalonder())
        self.desc_diff_thresh = 0.001
      elif DESCRIPTOR=='SAD':
        self.vo = Tracker(self.cam)
        self.desc_diff_thresh = 2000.0
      else:
        print "Unknown descriptor"
        return


################### IMAGE CALLBACK ############################
  def frame(self, imarray):

    # No calibration params yet.
    if not self.vo:
      return

    if self.seq > 10000:
      sys.exit()
    if DEBUG:
      print ""
      print ""
      print "Frame ", self.seq
      print ""
      print ""


    im = imarray.images[1]
    im_r = imarray.images[0]
    if im.colorspace == "mono8":
      im_py = Image.fromstring("L", (im.width, im.height), im.data)
      im_r_py = Image.fromstring("L", (im_r.width, im_r.height), im_r.data)
    elif im.colorspace == "rgb24":
      use_color = True
      im_col_py = Image.fromstring("RGB", (im.width, im.height), im.data)
      im_py = im_col_py.convert("L")
      im_r_py = Image.fromstring("RGB", (im_r.width, im_r.height), im_r.data)
      im_r_py = im_r_py.convert("L")
    else :
      print "Unknown colorspace"
      return
    

    # Detect faces on the first frame
    if not self.current_keyframes :
      self.faces = self.p.detectAllFaces(im_py.tostring(), im.width, im.height, self.cascade_file, 1.0, None, None, True) 
      if DEBUG:
        print "Faces ", self.faces
      
    sparse_pred_list = []
    sparse_pred_list_2d = []
    old_rect = [0,0,0,0]
    ia = SparseStereoFrame(im_py,im_r_py)
    ia.matches = []
    ia.desc_diffs = []
    ia.good_matches = []

    # Track each face
    iface = -1
    for face in self.faces:
      
      iface += 1

      (x,y,w,h) = copy.copy(self.faces[iface])
      if DEBUG:
        print "A face ", (x,y,w,h)

      (old_center, old_diff) = self.rect_to_center_diff((x,y,w,h)) 
      
      if self.face_centers_3d and iface<len(self.face_centers_3d):
        censize3d = list(copy.copy(self.face_centers_3d[iface]))
        censize3d.append(2.0*self.real_face_sizes_3d[iface]) ###ZMULT
        self.get_features(ia, self.num_feats, (x,y,w,h), censize3d)
      else:
        self.get_features(ia, self.num_feats, (x, y, w, h), (0.0,0.0,0.0,1000000.0))
      if not ia.kp2d:
        continue

      # First frame:
      if len(self.current_keyframes) < iface+1:

        (cen,diff) = self.rect_to_center_diff((x,y,w,h))
        cen3d = self.cam.pix2cam(cen[0],cen[1],ia.avgd)
        cen3d = list(cen3d)
        ltf = self.cam.pix2cam(x,y,ia.avgd)
        rbf = self.cam.pix2cam(x+w,y+h,ia.avgd)
        fs3d = ( (rbf[0]-ltf[0]) + (rbf[1]-ltf[1]) )/4.0
        # This assumes that we're tracking the face plane center, not the center of the head sphere. 
        # If you want to track the center of the sphere instead, do: cen3d[2] += fs3d

        # Check that the face is a reasonable size. If not, skip this face.
        if 2*fs3d < self.min_real_face_size or 2*fs3d > self.max_real_face_size or iface > 1: #HACK: ONLY ALLOW ONE FACE
          self.faces.pop(iface)
          iface -= 1
          continue

        if DESCRIPTOR=='CALONDER':
          self.vo.collect_descriptors(ia)
        elif DESCRIPTOR=='SAD':
          self.vo.collect_descriptors_sad(ia)
        else:
          pass

        self.current_keyframes.append(0)
        self.keyframes.append(copy.copy(ia))

        self.feats_to_centers.append(self.make_face_model( cen, diff, ia.kp2d ))

        self.real_face_sizes_3d.append( copy.deepcopy(fs3d) )
        self.feats_to_centers_3d.append( self.make_face_model( cen3d, (fs3d,fs3d,fs3d), ia.kp3d) )
        self.face_centers_3d.append( copy.deepcopy(cen3d) )

        self.recent_good_frames.append(copy.copy(ia))
        self.recent_good_rects.append(copy.deepcopy([x,y,w,h]))
        self.recent_good_centers_3d.append(copy.deepcopy(cen3d))
        self.recent_good_motion.append([0.0]*3) #dx,dy,dimfacesize
        self.recent_good_motion_3d.append([0.0]*3)

        self.same_key_rgfs.append(True)
        
        if DEBUG:
          print "cen2d", cen
          print "cen3d", self.face_centers_3d[iface]

        # End first frame

      # Later frames
      else :
        if DESCRIPTOR=='CALONDER':
          self.vo.collect_descriptors(ia)
        elif DESCRIPTOR=='SAD':
          self.vo.collect_descriptors_sad(ia)
        else:
          pass


        done_matching = False
        bad_frame = False
        while not done_matching:

          # Try matching to the keyframe
          keyframe = self.keyframes[self.current_keyframes[iface]]
          temp_match = self.vo.temporal_match(ia,keyframe,want_distances=True)
          ia.matches = [(m2,m1) for (m1,m2,m3) in temp_match]
          ia.desc_diffs = [m3 for (m1,m2,m3) in temp_match]
          print "temp matches", temp_match       
          ia.good_matches = [s < self.desc_diff_thresh for s in ia.desc_diffs]

          n_good_matches = len([m for m in ia.desc_diffs if m < self.desc_diff_thresh])

          if DEBUG:
            if len(keyframe.kp)<2:
              print "Keyframe has less than 2 kps"
            if n_good_matches < len(keyframe.kp)/2.0:
              print "ngoodmatches, len key.kp, len key.kp/2", n_good_matches, len(keyframe.kp), len(keyframe.kp)/2.0
        
          # Not enough matches, get a new keyframe
          if len(keyframe.kp)<2 or n_good_matches < len(keyframe.kp)/2.0 : 

            if DEBUG:
              print "New keyframe"

            # Make a new face model, either from a recent good frame, or from the current image
            if not self.same_key_rgfs[iface] :

              if DEBUG:
                print "centers at beginning of new keyframe"
                print "cen2d", [self.faces[iface][0]+self.faces[iface][2]/2.0, self.faces[iface][1]+self.faces[iface][3]/2.0]
                print "cen3d", self.face_centers_3d[iface]


              matched_z_list = [kp3d[2] for (kp3d,is_good) in zip(self.recent_good_frames[iface].kp3d,self.recent_good_frames[iface].good_matches) if is_good]
              if len(matched_z_list) == 0:
                matched_z_list = [kp3d[2] for kp3d in self.recent_good_frames[iface].kp3d]
              avgz_goodmatches = sum(matched_z_list)/ len(matched_z_list)
              tokeep = [math.fabs(self.recent_good_frames[iface].kp3d[i][2]-avgz_goodmatches) < 2.0*self.real_face_sizes_3d[iface] 
                        for i in range(len(self.recent_good_frames[iface].kp3d))]
              kp3d_for_model = [kp3d for (kp3d,tk) in zip(self.recent_good_frames[iface].kp3d,tokeep) if tk]
              kp_for_model = [kp for (kp,tk) in zip(self.recent_good_frames[iface].kp,tokeep) if tk]
              # If you're not left with enough points, just take all of them and don't worry about the depth constraints.
              if len(kp3d_for_model) < 2:
                kp3d_for_model = copy.deepcopy(self.recent_good_frames[iface].kp3d)
                kp_for_model = copy.deepcopy(self.recent_good_frames[iface].kp)

              (cen, diff) = self.rect_to_center_diff(self.recent_good_rects[iface])
              self.feats_to_centers[iface] = self.make_face_model( cen, diff, [(kp0,kp1) for (kp0,kp1,kp2) in kp_for_model])

              cen3d = self.recent_good_centers_3d[iface]
              self.feats_to_centers_3d[iface] = self.make_face_model( cen3d, [self.real_face_sizes_3d[iface]]*3, kp3d_for_model)

              self.keyframes[self.current_keyframes[iface]] = copy.copy(self.recent_good_frames[iface])
              self.keyframes[self.current_keyframes[iface]].kp = kp_for_model
              self.keyframes[self.current_keyframes[iface]].kp2d = [(k0,k1) for (k0,k1,k2) in kp_for_model]
              self.keyframes[self.current_keyframes[iface]].kp3d = kp3d_for_model
              self.keyframes[self.current_keyframes[iface]].matches = [(i,i) for i in range(len(kp_for_model))]
              self.keyframes[self.current_keyframes[iface]].good_matches = [True]*len(kp_for_model)
              self.keyframes[self.current_keyframes[iface]].desc_diffs = [0]*len(kp_for_model)

              if DESCRIPTOR=='CALONDER':
                self.vo.collect_descriptors(self.keyframes[self.current_keyframes[iface]])
              elif DESCRIPTOR=='SAD':
                self.vo.collect_descriptors_sad(self.keyframes[self.current_keyframes[iface]])
              else:
                pass
              

              self.face_centers_3d[iface] = copy.deepcopy(cen3d)
              # Not changing the face size

              self.current_keyframes[iface] = 0 #### HACK: ONLY ONE KEYFRAME!!!

              self.same_key_rgfs[iface] = True
              # Don't need to change the recent good frame yet.

              if DEBUG:
                print "centers at end of new keyframe"
                print "cen2d", [self.faces[iface][0]+self.faces[iface][2]/2.0, self.faces[iface][1]+self.faces[iface][3]/2.0]
                print "cen3d", self.face_centers_3d[iface]

            else :

              # Making a new model off of the current frame but with the predicted new position. 
              # HACK: The displacement computation assumes that the robot/head is still, fix this.
              bad_frame = True
              #done_matching = True
              if DEBUG:
                print "Bad frame ", self.seq, " for face ", iface

              (cen,diff) = self.rect_to_center_diff(self.faces[iface])
              if DEBUG:
                print "Motion for bad frame ", self.recent_good_motion[iface], self.recent_good_motion_3d[iface]
              new_cen = [cen[0]+self.recent_good_motion[iface][0], cen[1]+self.recent_good_motion[iface][1]]
              diff = [diff[0]+self.recent_good_motion[iface][2], diff[1]+self.recent_good_motion[iface][2]]                 

              self.faces[iface] = (new_cen[0]-diff[0], new_cen[1]-diff[1], 2.0*diff[0], 2.0*diff[1])
              (x,y,w,h) = copy.deepcopy(self.faces[iface])

              pred_cen_3d = [o+n for (o,n) in zip(self.face_centers_3d[iface],self.recent_good_motion_3d[iface])]
              pred_cen_3d.append(2.0*self.real_face_sizes_3d[iface])  #### ZMULT
              self.get_features(ia, self.num_feats, (x,y,w,h), pred_cen_3d)
              if not ia.kp2d:
                break

              if DESCRIPTOR=='CALONDER':
                self.vo.collect_descriptors(ia)
              elif DESCRIPTOR=='SAD':
                self.vo.collect_descriptors_sad(ia)
              else:
                pass


              self.keyframes[self.current_keyframes[iface]] = copy.copy(ia)
              self.current_keyframes[iface] = 0
              (cen,diff) = self.rect_to_center_diff(self.faces[iface])
              self.feats_to_centers[iface] = self.make_face_model( cen, diff, ia.kp2d )
              self.feats_to_centers_3d[iface] = self.make_face_model( [pred_cen_3d[0],pred_cen_3d[1],pred_cen_3d[2]], [self.real_face_sizes_3d[iface]]*3, ia.kp3d)
              self.face_centers_3d[iface] = copy.deepcopy(pred_cen_3d)
              self.same_key_rgfs[iface] = True

          # Good matches, mark this frame as good
          else:
            done_matching = True

          # END MATCHING


        # If we got enough matches for this frame, track.
        if ia.kp and ia.kp2d:

          # Track
          sparse_pred_list = []
          sparse_pred_list_2d = []
          probs = []
          bandwidths = []
          size_mult = 0.05 #1.0

          for ((match1, match2), score) in zip(ia.matches, ia.desc_diffs):
            if score < self.desc_diff_thresh:
              sparse_pred_list.append( [ia.kp3d[match2][i]+self.feats_to_centers_3d[iface][match1][i] for i in range(3)] )
              sparse_pred_list_2d.append( [ia.kp2d[match2][i]+self.feats_to_centers[iface][match1][i] for i in range(2)] )
              #probs.append(score)
          probs = [1.0] * len(sparse_pred_list_2d) # Ignore actual match scores. Uncomment line above to use the match scores.
          bandwidths = [size_mult*self.real_face_sizes_3d[iface]] * len(sparse_pred_list_2d)

          (old_center, old_diff) = self.rect_to_center_diff(self.faces[iface])
 
          if DEBUG:
            print "Old center 3d ", self.face_centers_3d[iface]
            print "Old center 2d ", old_center

          old_rect = self.faces[iface] # For display only

          new_center = self.mean_shift_sparse( self.face_centers_3d[iface][0:3], sparse_pred_list, probs, bandwidths, 10, 1.0 )
          new_center_2d = self.cam.cam2pix(new_center[0], new_center[1], new_center[2]) 
          # The above line assumes that we're tracking the face plane center, not the center of the head sphere. 
          # If you want to track the center of the sphere instead, subtract self.real_face_sizes[iface] from the z-coord.
          ltf = self.cam.cam2pix( new_center[0]-self.real_face_sizes_3d[iface], new_center[1]-self.real_face_sizes_3d[iface], new_center[2])
          rbf = self.cam.cam2pix( new_center[0]+self.real_face_sizes_3d[iface], new_center[1]+self.real_face_sizes_3d[iface], new_center[2])
          w = rbf[0]-ltf[0]
          h = rbf[1]-ltf[1]       

          if DEBUG:
            print "new center 3d ", new_center
            print "new_center 2d ", new_center_2d

          (nx,ny,nw,nh) = (new_center_2d[0]-(w-1)/2.0, new_center_2d[1]-(h-1)/2.0, w, h)

          # Force the window back into the image.
          nx += max(0,0-nx) + min(0, im.width - nx+nw)
          ny += max(0,0-ny) + min(0, im.height - ny+nh)

          self.faces[iface] = [nx, ny, nw, nh]
          self.recent_good_rects[iface] = [nx,ny,nw,nh]
          self.recent_good_centers_3d[iface] = copy.deepcopy(new_center)
          if bad_frame:
            self.recent_good_motion[iface] = self.recent_good_motion[iface]
            self.recent_good_motion_3d[iface] = self.recent_good_motion_3d[iface]
          else:
            self.recent_good_motion[iface] = [new_center_2d[0]-old_center[0], new_center_2d[1]-old_center[1], ((nw-1.0)/2.0)-old_diff[0]]
            self.recent_good_motion_3d[iface] = [ new_center[i]-self.face_centers_3d[iface][i] for i in range(len(new_center))]
          self.face_centers_3d[iface] = copy.deepcopy(new_center)
          self.recent_good_frames[iface] = copy.copy(ia)
          self.same_key_rgfs[iface] = False


          if DEBUG:
            print "motion ", self.recent_good_motion[iface], self.recent_good_motion_3d[iface]
            print "face 2d ", self.faces[iface]
            print "face center 3d ", self.face_centers_3d[iface]


          # Output the location of this face center in the 3D camera frame (of the left camera), and rotate 
          # the coordinates to match the robot's idea of the 3D camera frame.
          center_uvd = (nx + (nw-1)/2.0, ny + (nh-1)/2.0, (numpy.average(ia.kp,0))[2] )
          center_camXYZ = self.cam.pix2cam(center_uvd[0], center_uvd[1], center_uvd[2])
          center_robXYZ = (center_camXYZ[2], -center_camXYZ[0], -center_camXYZ[1])

          ########### PUBLISH the face center for the head controller to track. ########
          if not self.usebag:
            #stamped_point = PointStamped()
            #(stamped_point.point.x, stamped_point.point.y, stamped_point.point.z) = center_robXYZ
            #stamped_point.header.frame_id = "stereo"
            #stamped_point.header.stamp = imarray.header.stamp
            #self.pub.publish(stamped_point)
            pm = PositionMeasurement()
            pm.header.stamp = imarray.header.stamp
            pm.name = "stereo_face_feature_tracker"
            pm.object_id = -1
            (pm.pos.x,pm.pos.y, pm.pos.z) = center_robXYZ
            pm.header.frame_id = "stereo_link"
            pm.reliability = 0.5;
            pm.initialization = 0;
            #pm.covariance
            self.pub.publish(pm)            
    

        # End later frames


      ############ DRAWING ################
      if SAVE_PICS:

        if not self.keyframes or len(self.keyframes) <= iface :
          bigim_py = im_py
          draw = ImageDraw.Draw(bigim_py)
        else :
          key_im = self.keyframes[self.current_keyframes[iface]]
          keyim_py = Image.fromstring("L", key_im.size, key_im.rawdata)
          bigim_py = Image.new("RGB",(im_py.size[0]+key_im.size[0], im_py.size[1]))
          bigim_py.paste(keyim_py.convert("RGB"),(0,0))
          bigim_py.paste(im_py,(key_im.size[0]+1,0))
          draw = ImageDraw.Draw(bigim_py)

          (x,y,w,h) = self.faces[iface]
          draw.rectangle((x,y,x+w,y+h),outline=(0,255,0))
          draw.rectangle((x+key_im.size[0],y,x+w+key_im.size[0],y+h),outline=(0,255,0))
          (x,y,w,h) = old_rect
          draw.rectangle((x,y,x+w,y+h),outline=(255,255,255))
          draw.rectangle((x+key_im.size[0],y,x+w+key_im.size[0],y+h),outline=(255,255,255))

          mstart = old_center
          mend = (old_center[0]+self.recent_good_motion[iface][0], old_center[1]+self.recent_good_motion[iface][1])
          draw.rectangle((mstart[0]-1,mstart[1]-1,mstart[0]+1,mstart[1]+1), outline=(255,255,255))
          draw.rectangle((mend[0]-1,mend[1]-1,mend[0]+1,mend[1]+1), outline=(0,255,0))
          draw.line(mstart+mend, fill=(255,255,255))

          for (x,y) in key_im.kp2d :
            draw_x(draw, (x,y), (1,1), (255,0,0))
          for (x,y) in ia.kp2d:
            draw_x(draw, (x+key_im.size[0],y), (1,1), (255,0,0))

          if self.seq > 0 :

            if ia.matches:
              for ((m1,m2), score) in zip(ia.matches,ia.desc_diffs) :
                if score > self.desc_diff_thresh :
                  color = (255,0,0)
                else :
                  color = (0,255,0)
                draw.line((key_im.kp2d[m1][0], key_im.kp2d[m1][1], ia.kp2d[m2][0]+key_im.size[0], ia.kp2d[m2][1]), fill=color)

            for (i, (u,v)) in enumerate(sparse_pred_list_2d) :
              bscale = min(1,probs[i]/0.01)
              draw_x(draw, (u,v), (1,1), (128.0+128.0*bscale,128.0+128.0*bscale,(1.0-bscale)*255.0))
              draw_x(draw, (u+key_im.size[0],v), (1,1),(128.0+128.0*bscale,128.0+128.0*bscale,(1.0-bscale)*255.0))      
 

          ####### PUBLISH 3d visualization point cloud ###################
          if self.usebag and self.visualize:
            cloud = PointCloud()
            cloud.header.frame_id = "stereo"
            cloud.header.stamp = imarray.header.stamp
            cloud.points = []
            cloud.points.append(Point())
            (cloud.points[0].x, cloud.points[0].y, cloud.points[0].z) = self.face_centers_3d[iface][:3]
            for (i,kp3d) in enumerate(ia.kp3d):
              cloud.points.append(Point())
              (cloud.points[i].x,cloud.points[i].y,cloud.points[i].z) = kp3d

            lp = len(cloud.points)
            if self.seq > 0:
              for (i, (u,v)) in enumerate(sparse_pred_list):
                cloud.points[lp+i].append(Point())
                (cloud.points[lp+i].x,cloud.points[lp+i].y,cloud.points[lp+i].z) = sparse_pred_list[i][:3]
                        
            self.pub.publish(cloud)

        bigim_py.save("/tmp/tiff/feats%06d_%03d.tiff" % (self.seq, iface))
        #END DRAWING


      # END FACE LOOP

    self.seq += 1

    


############# MAIN #############
def main(argv) :
    
  people_tracker = PeopleTracker()

  if len(argv)>0 and argv[0]=="-bag":
      people_tracker.usebag = True
   #   if len(argv)>1 and argv[1]=="-visualize":
   #     people_tracker.visualize = True

  # Use a bag file
  if people_tracker.usebag :
  
    import rosrecord
    #filename = "/wg/stor2/prdata/videre-bags/people-color-close-single-2.bag"
    filename = "/wg/stor2/prdata/videre-bags/loop1-mono.bag"
    #filename = "/wg/stor2/prdata/videre-bags/face2.bag"

    if SAVE_PICS:
      try:
        os.mkdir("/tmp/tiff/")
      except:
        pass

  #  if people_tracker.visualize:
  #    people_tracker.pub = rospy.Publisher('sensor_msgs/full_cloud',PointCloud)
  #    rospy.init_node('videre_face_tracker',anonymous=True)

    num_frames = 0
    start_frame = 4700
    end_frame = 5000
    for topic, msg in rosrecord.logplayer(filename):
      if topic == '/videre/cal_params':
        people_tracker.params(msg)
      elif topic == '/videre/images':
        if num_frames >= start_frame and num_frames < end_frame:
          people_tracker.frame(msg)
          if people_tracker.visualize:
            s = raw_input("Press any key in this window to proceed to the next frame.")
        elif num_frames >= end_frame:
          break
        num_frames += 1
      else :
        pass

  # Use new ROS messages, and output the 3D position of the face in the camera frame.
  else :
    print "Non-bag"

    if SAVE_PICS:
      try:
        os.mkdir("/tmp/tiff/")
      except:
        pass

    #people_tracker.pub = rospy.Publisher('head_controller/track_point',PointStamped)
    people_tracker.pub = rospy.Publisher('/stereo_face_feature_tracker/position_measurement',PositionMeasurement)
    rospy.init_node('stereo_face_feature_tracker', anonymous=True)
    #rospy.Subscriber('/head_controller/track_point',PointStamped,people_tracker.point_stamped)
    rospy.Subscriber('/videre/images',ImageArray,people_tracker.frame)
    rospy.Subscriber('/videre/cal_params',String,people_tracker.params)
    rospy.spin()


if __name__ == '__main__' :

  if False:
    import hotshot
    prof = hotshot.Profile("hotshot_directed")
    prof.runcall(main,sys.argv[1:])
    prof.close()
  else:
    main(sys.argv[1:])
