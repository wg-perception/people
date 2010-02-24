(in-package :lanes)

(defvar *status* nil)

(def-ros-node fake-person-sender () (:spin t)
  (subscribe "lane_changer/status" "std_msgs/Byte" #'status-callback)
  (register-service "is_person_on_path" 'PersonOnPath))

(def-service-callback PersonOnPath ()
  (make-response :value (eql *status* 1)))

(defun status-callback (status)
  (with-fields (data) status
    (setq *status* data)))