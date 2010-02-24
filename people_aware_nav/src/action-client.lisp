(in-package :lane-following)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Simple action client for move base
;; Single-goal
;; Not threadsafe
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *inc* .5)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; API
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (action-client (:conc-name nil) (:constructor make-ac)) 
  status goal-pub cancel-pub goal-type)

(defun make-action-client (topic-prefix goal-type)
  "Return an action client, on which send-blocking-goal can be called."
  (let ((client
	 (make-ac :status nil
		  :goal-pub (advertise (fully-qualified-name topic-prefix "goal") goal-type)
		  :cancel-pub (advertise (fully-qualified-name topic-prefix "cancel") 
					 "actionlib_msgs/GoalID")
		  :goal-type goal-type)))
    (subscribe (fully-qualified-name topic-prefix "status") "actionlib_msgs/GoalStatusArray" 
	       (store-message-in (status client)))
    client))


(defun initiate-action (ac goal)
  "Send goal to action client and return its id"
  (let* ((id (generate-goal-id))
	 (goal-id-msg (make-msg "actionlib_msgs/GoalID" :stamp (ros-time) :id id))
	 (m (make-msg (goal-type ac) :goal goal :goal_id goal-id-msg)))
    (ros-debug action-client "Sending goal with id ~a" id)
    (publish (goal-pub ac) m)
    id))
    
(defun cancel-action (ac id)
  "Send a cancel message for the given goal id"
  (publish (cancel-pub ac) (make-msg "actionlib_msgs/GoalID" :stamp (ros-time) :id id)))

(defun cancel-all-actions (ac)
  "Send an empty cancel message, which means cancel all goals"
  (publish (cancel-pub ac) (make-msg "actionlib_msgs/GoalID")))

(defun block-on-goal (ac id &optional timeout)
  "Wait till the status of this goal, as computed by goal-status, is :succeeded, :aborted, or :rejected (in which case that status is returned), or until the timeout, in which case :timeout is returned.  Cancels the goal before returning (in the normal scheme of things, that will be a noop)."
  (unwind-protect
  (loop-at-most-every *inc*
     (when (and timeout (< (decf timeout *inc*) 0))
       (ros-info action-client "Goal ~a timing out" id)
       (return :timeout))
     (let ((status (goal-status ac id)))
       (ros-debug action-client "Status is ~a" status)
       (when (member status '(:succeeded :aborted :rejected))
	 (return status))))
    (cancel-action ac id)))


(defun send-blocking-goal (ac goal &optional timeout)
  "Send the goal to the action client.  Does an initiate-action followed by block-on-goal."
  (let ((id (initiate-action ac goal)))
    (block-on-goal ac id timeout)))

(defun goal-status (ac id)
  "Return the status of this goal (:succeeded, :pending, etc) or :not-found if it's not on the current status list, or :empty-status if the status message itself hasn't been set yet"
  (if (status ac)
      (with-fields (status_list) (status ac)
	(let ((item-status (find-if (status-id-matcher id) status_list)))
	  (if item-status
	      (lookup-status-code (actionlib_msgs-msg:status-val item-status))
	      :not-found)))
      :empty-status))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *status-codes* (symbol-codes 'actionlib_msgs-msg:<GoalStatus>))

(defun lookup-status-code (i)
  (or (car (rassoc i *status-codes*)) :not-found))


(defvar *goal-id* 0)

(defun generate-goal-id ()

  (format nil "~a-~a-~a" *ros-node-name* (incf *goal-id*) (roslisp:ros-time)))


(defun status-id-matcher (id)
  #'(lambda (status)
      (with-fields ((status-id (id goal_id))) status
	(equal status-id id))))

