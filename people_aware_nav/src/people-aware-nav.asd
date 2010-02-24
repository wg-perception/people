;;;; -*- Mode: LISP -*-


(in-package :asdf)	

(defsystem "people_aware_nav/people-aware-nav"
  :name "people-aware-nav"
  :components
  ((:file "transform-2d")
   (:file "lanes" :depends-on ("transform-2d" "action-client"))
   (:file "pkg" :depends-on ("transform-2d"))
   (:file "action-client" :depends-on ("pkg"))
   (:file "fake-person" :depends-on ("pkg")))
  :depends-on (:roslisp :geometry_msgs-msg :sensor_msgs-msg  :people-msg 
			:move_base_msgs-msg :actionlib_msgs-msg
			:people_aware_nav-msg :people_aware_nav-srv :roslisp-utils))

;;;; eof
