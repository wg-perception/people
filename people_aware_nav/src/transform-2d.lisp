(defpackage :transform-2d
  (:use :cl)
  (:documentation "Operations on rigid transformations in 2d")
  (:export

   ;; creation
   :make-pose
   :make-rigid-transformation

   ;; types
   :point
   :pose
   :rigid-transformation

   ;; accessors
   :pose-position
   :pose-orientation

   ;; ops
   :transform-point
   :transform-between
   :transform-pose
   :inverse
   :mv*
   :a-
   :inner-product
   :unit-vector
   :rotation-matrix
   :vector-angle
))


(in-package :transform-2d)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (rigid-transformation (:conc-name nil) (:constructor make-rigid-transformation (offset angle)))
  offset angle)

(deftype point ()
  '(vector float 2))

(deftype matrix ()
  '(array float (2 2)))

(defstruct (pose (:constructor make-pose (position orientation)))
  position orientation)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Transforms
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun transform-point (trans point)
  (declare (rigid-transformation trans) (point point))
  (a+ (mv* (rotation-matrix (angle trans)) point) (offset trans)))

(defun transform-pose (trans pose)
  (declare (rigid-transformation trans) (pose pose))
  (make-pose
   (transform-point trans (pose-position pose))
   (mod (+ (pose-orientation pose) (angle trans)) (* 2 pi))))

(defun transform-between (pose1 pose2)
  "Return the unique rigid transformation sending pose1 to pose2"
  (declare (pose pose1) (pose pose2))
  (let ((theta (mod (- (pose-orientation pose2) (pose-orientation pose1)) (* 2 pi)))
	(position1 (pose-position pose1))
	(position2 (pose-position pose2)))
    (let ((x1 (aref position1 0)) (y1 (aref position1 1))
	  (x2 (aref position2 0)) (y2 (aref position2 1))
	  (c (cos theta)) (s (sin theta)))
      (make-rigid-transformation (vector (+ x2 (* s y1) (- (* c x1)))
					 (- y2 (* c y1) (* s x1)))
				 theta))))

(defun inverse (trans)
  (declare (rigid-transformation trans))
  (let ((c (cos (angle trans))) (s (sin (angle trans)))
	(dx (aref (offset trans) 0)) (dy (aref (offset trans) 1)))
    (make-rigid-transformation (vector (- (+ (* c dx) (* s dy))) (- (* s dx) (* c dy))) (mod (- (angle trans)) (* 2 pi)))))
    
	  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Matrix and vector ops
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun rotation-matrix (theta)
  (declare (float theta))
  (let ((a (make-array '(2 2) :element-type 'float :initial-element 0.0))
	(c (cos theta))
	(s (sin theta)))
    (setf (aref a 0 0) c
	  (aref a 0 1) (- s)
	  (aref a 1 0) s
	  (aref a 1 1) c)
    a))


(defun unit-vector (v)
  (declare (point v))
  (let ((n (l2-norm v)))
    (assert (not (zerop n)) nil "Attempted to find unit vector in direction of zero-vector")
    (vector (/ (aref v 0) n) (/ (aref v 1) n))))

(defun l2-norm (v)
  (declare (point v))
  (sqrt (inner-product v v)))

(defun inner-product (v w)
  (declare (point v) (point w))
  (+ (* (aref v 0) (aref w 0)) (* (aref v 1) (aref w 1))))

(defun mv* (m v)
  (declare (matrix m) (point v))
  (vector (+ (* (aref m 0 0) (aref v 0))
	     (* (aref m 0 1) (aref v 1)))
	  (+ (* (aref m 1 0) (aref v 0))
	     (* (aref m 1 1) (aref v 1)))))

(defun a+ (&rest args)
  (apply #'map 'vector #'+ args))

(defun a- (&rest args)
  (apply #'map 'vector #'- args))

(defun vector-angle (v)
  (declare (point v))
  (atan (aref v 1) (aref v 0)))