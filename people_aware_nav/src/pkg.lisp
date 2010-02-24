(defpackage :lane-following
  (:nicknames :lanes)
  (:use :roslisp :cl :transform-2d :sb-thread :roslisp-queue :people_aware_nav-srv :sb-thread)
  (:export :main :test-move-right))


