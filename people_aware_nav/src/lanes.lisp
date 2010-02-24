(in-package :lane-following)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Params
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *robot-radius* .32 "Circumscribed radius of robot in metres")
(defparameter *wall-buffer* .2 "Additional buffer distance that we'd like to keep from wall")
(defvar *person-on-path-use-stub* nil)
(defvar *global-frame*)
(defparameter *angle-offset* (+ (/ pi 2) .6))
(defparameter *offset-length* 3.5)
(defparameter *move-right-sharpness* 2)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (hallway (:constructor make-hallway (transform width)) (:type list))
  transform width)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; State
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *robot-pose* nil "Robot's current 3d pose.  Set by pose-callback.")
(defvar *hallway* nil "Configuration of the hallway we're in.  Set by hallway-callback.")
(defvar *current-goal* '(0 0) "Current nav goal.  Set by send-move-goal.")
(defvar *new-goal* (make-queue :max-size 1) "A new nav goal received on the goal topic.  Set by goal-callback and by main")
(defvar *person-position* nil "Person's last observed position")
(defvar *person-position* nil "Person's last observed position")
(defvar *status* :initial "Status of the executive")
(defvar *action-client* :initial "Move base action client")
(defvar *status-pub* nil "Status publication object")
(defvar *goal-pub* nil "Goal publication object")
(defvar *move-base-result*)
(defvar *mutex* (make-mutex :name "hallway"))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun main ()
  (with-ros-node ("lane_changer")
    (setup-node)
    (with-parallel-thread (status-publisher :hallway-status-publisher)
      (loop (apply #'goto (dequeue-wait *new-goal*))))))


(defun setup-node ()
  (subscribe "hallway_points" "sensor_msgs/PointCloud" #'hallway-callback)
  (subscribe "goal" "geometry_msgs/PoseStamped" #'goal-callback)
  (subscribe "face_detector/people_tracker_measurements" "people/PositionMeasurement" (store-message-in *person-position*))
  (subscribe "robot_pose" "geometry_msgs/Pose2D" #'pose-callback)
  (setq *status-pub* (advertise "~status" "std_msgs/Byte")
	*goal-pub* (advertise "~goal" "geometry_msgs/PoseStamped")
	*global-frame* (get-param "hallway_global_frame_id" "/map")
	*person-on-path-use-stub* (get-param "~person_on_path_use_stub" *person-on-path-use-stub*)
	*action-client* (make-action-client "move_base" "move_base_msgs/MoveBaseActionGoal")
	*hallway* nil
	*robot-pose* nil))
  

(defun goto (x y theta)
  (ros-info pan "Initiating hallway move to goal ~a ~a ~a" x y theta)
  (setq *status* :move)
  (unwind-protect
       (let ((id (initiate-move x y theta)))
	 (loop-at-most-every 1
	      (cond
		((member (goal-status *action-client* id) '(:succeeded :aborted))
		 (return-from goto (goal-status *action-client* id)))
		((not (queue-empty *new-goal*)) (return-from goto :preempted))
		((person-on-path) (return))))

	 ;; If person is on path
	 (cancel-all-actions *action-client*)
	 (setq *status* :move-right)
	 (move-to-right)

	 (setq *status* :contrained-move
	       id (initiate-move x y theta :constrained t))
	 (loop-at-most-every 1
	      (cond
		((member (goal-status *action-client* id) '(:succeeded :aborted))
		 (return-from goto (goal-status *action-client* id)))
		((not (queue-empty *new-goal*)) (return-from goto :preempted)))))

    ;; Cleanup: always disable nav before exiting
    (cancel-all-actions *action-client*)
    ))

(defun status-publisher ()
  (loop-at-most-every .5
     (let ((status-byte (ecase *status* (:initial 0) (:move 1) (:move-right 2) (:constrained-move 3))))
       (publish *status-pub* (make-message "std_msgs/Byte" :data status-byte)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Callouts
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun move-to-right ()
  (ros-info pan "Moving to right")
  (let* ((pose (waiting-pose *robot-pose* *hallway*))
	 (position (pose-position pose))
	 (theta (pose-orientation pose))
	 (id (initiate-move (aref position 0) (aref position 1) theta)))
#|  (sleep 5)
    (with-fields ((frame (frame_id header)) (stamp (stamp header)) (pos pos)) *person-position*
      (call-service "glance_at" 'glanceat :point_stamped (make-message "geometry_msgs/PointStamped" 
								      (frame_id header) frame
								      (stamp header) stamp
								      point pos))) |#
    (block-on-goal *action-client* id)))




(defun person-on-path ()
  (if *person-on-path-use-stub*
      (y-or-n-p "Is person on path?")
      (let ((v (not (= 0 (people_aware_nav-srv:value-val (call-service "is_person_on_path" 'people_aware_nav-srv:PersonOnPath))))))
	(when v (format t "~&Person is on path at time ~a" (ros-time)))
	v)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Callbacks
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *abort* (symbol-code 'robot_actions-msg:<ActionStatus> :aborted))
(defparameter *succeed* (symbol-code 'robot_actions-msg:<ActionStatus> :success))

(defun hallway-callback (m)
  
  (with-fields (points) m
    (if (= (length points) 3)

	(setf *hallway* (hallway-info (make-point (aref points 0))
				      (make-point (aref points 1))
				      (make-point (aref points 2))))

	(ros-error pan "Hallway cloud ~a had incorrect length.  Skipping." points))))


(defun goal-callback (m)
  (publish *goal-pub* m)
  (with-fields ((frame (frame_id header)) 
		(x (x position pose)) 
		(y (y position pose))
		(w (w orientation pose))) m
    (ros-info pan "Received goal ~a, ~a in frame ~a" x y frame)
    (let ((theta (* 2 (acos w))))
      (when (not (eql (aref frame 0) #\/))
	(setq frame (concatenate 'string "/" frame)))
      (if (search frame *global-frame*)
	  (enqueue (list x y theta) *new-goal*)
	  (ros-error pan "Ignoring goal ~a ~a ~a as frame id ~a does not equal ~a"
		     x y theta frame *global-frame*)))))
  

(defun pose-callback (pose)
  (with-fields (x y theta) pose
    (setq *robot-pose* (make-pose (vector x y) theta))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; teleop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun rotate (dtheta)
  (send-blocking-goal *action-client* (make-nav-goal 0 0 dtheta "base_link")))

(defun forward (dx)
  (send-blocking-goal *action-client* (make-nav-goal dx 0 0 "base_link")))

(defun strafe (dy)
  (send-blocking-goal *action-client* (make-nav-goal 0 dy 0 "base_link")))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Testing
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun test-move-right ()
  (with-ros-node ("move_right_test")
    (setup-node)
    (let ((pub (advertise "move_right_status" "std_msgs/String")))
      (spin-until (and *hallway* *robot-pose* (status *action-client*)) 1.0 
		  (ros-info test-move-right "Action client status is ~a~&Hallway is ~a~&Robot-pose is ~a" 
			    (status *action-client*) *hallway* *robot-pose*))

      (with-parallel-thread (move-right-monitor)
	(move-to-right)
	(publish-msg pub :data "success")))))

(defun move-right-monitor ()
  (loop-at-most-every 1.0
       (ros-info test-move-right "Position is ~a~&Status is ~a" 
		 (pose-position *robot-pose*)
		 (status *action-client*))))
      
      

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun initiate-move (x y th &key (constrained nil))
  (call-service "/move_base/set_nav_constraint" 'SetNavConstraint
		:forbidden (make-boundary (make-pose (vector x y) th) constrained))
  (initiate-action *action-client* (make-nav-goal x y th)))

(defun make-nav-goal (x y &optional (th 0) (frame *global-frame*))
  (make-msg "move_base_msgs/MoveBaseGoal"
	    (x position pose target_pose) x
	    (y position pose target_pose) y
	    (w orientation pose target_pose) (cos (/ th 2))
	    (z orientation pose target_pose) (sin (/ th 2))
	    (frame_id header target_pose) frame))


(defun hallway-info (p1 p2 p3)
  "Return 1) the transform from map to hallway frames 2) the hallway width, given that p1 and p2 are points on the left wall, and p3 is on the right wall"
  (declare (point p1) (point p2) (point p3))
  (let* ((d1 (a- p2 p1))
	 (d2 (a- p3 p1))
	 (length (inner-product d2 (unit-vector (mv* (rotation-matrix (/ pi -2)) d1))))
	 (flip (< length 0)))
    (make-hallway (transform-between (make-pose p1 (vector-angle d1)) (make-pose (vector 0.0 0.0) (/ pi (if flip -2 2)))) (abs length))))


(defun waiting-pose (current-pose hallway)
  "Return the pose corresponding to 'shifting to the right lane' in the map frame"
  (declare (pose current-pose) (values pose))
  (ros-info pan "Computing waiting pose given current pose ~a" current-pose)
  (let* ((hallway-pose (transform-pose (hallway-transform hallway) current-pose))
	 (hallway-frame-position (pose-position hallway-pose))
	 (facing-forward (<= 0.0 (pose-orientation hallway-pose) pi))
	 (target-wall-offset (if facing-forward
				 (- (hallway-width hallway) *robot-radius* *wall-buffer*)
				 (+ *robot-radius* *wall-buffer*)))
	 (target-wall-y (+ (aref hallway-frame-position 1)
			   (* (if facing-forward 1 -1)
			      *move-right-sharpness*
			      (abs (- target-wall-offset (aref hallway-frame-position 0))))))
	 (global-pose (transform-pose (inverse (hallway-transform hallway)) 
				      (make-pose (vector target-wall-offset target-wall-y) 
						 (/ pi (if facing-forward 2 -2))))))
    (ros-info pan "Waiting pose is ~a" global-pose)
    global-pose))

    
(defun make-point (p)
  (with-fields (x y) p
    (vector x y)))


(defun make-boundary (pose constrained?)
  (if constrained?
      (let ((pos (pose-position pose))
	    (theta (pose-orientation pose)))
	(make-message "geometry_msgs/Polygon"
		      :points (vector (get-offset-point pos (+ theta *angle-offset*))
				      (get-offset-point pos (+ theta pi))
				      (get-offset-point pos (- theta *angle-offset*)))))
      ;; if not constrained, return an empty polygon
      (make-message "geometry_msgs/Polygon")))



(defun get-offset-point (pos theta)
  (let ((x (aref pos 0))
	(y (aref pos 1)))
    (make-message "geometry_msgs/Point32"
		  :x (+ x (* *offset-length* (cos theta)))
		  :y (+ y (* *offset-length* (sin theta))))))