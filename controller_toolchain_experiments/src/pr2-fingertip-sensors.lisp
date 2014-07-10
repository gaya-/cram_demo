;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to
;;; endorse or promote products derived from this software without specific
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :controller-experiments)

;;;
;;; COMMON LISP UTILS
;;;

(defun merge-hash-tables (&rest tabels)
  (case (length tabels)
    (0 nil)
    (1 (first tabels))
    (t (reduce 
        (lambda (union tabel) 
          (maphash (lambda (key value) (setf (gethash key union) value)) tabel)
          union)
        (rest tabels)
        :initial-value (alexandria:copy-hash-table (first tabels))))))

;;;
;;; FLUENT NETWORKS COMPUTING STUFF WITH FINGERTIP READINGS
;;; AND SENSOR DESCRIPTIONS
;;;

(defparameter *l-gripper-input-fluent*
  (cpl-impl:make-fluent :value nil))
(defparameter *r-gripper-input-fluent*
  (cpl-impl:make-fluent :value nil))

(defparameter *l-gripper-stage1-fluent*
  (cpl-impl:fl-funcall #'pr2-fingertip-msg->map *l-gripper-input-fluent* "l_gripper"))

(defparameter *l-gripper-stage2-fluent*
  (cpl-impl:fl-funcall #'raw-fingertip-readings->newtons *l-gripper-stage1-fluent* (pr2-fintertip-description)))

(defun raw-fingertip-readings->newtons (sensor-readings sensor-descr)
  (let ((result (alexandria:copy-hash-table sensor-readings)))
    (maphash 
     (lambda (key value)
       (when (contains-association-p sensor-descr key) ; is sensor?
         (add-associations 
          result 
          key (/ value (get-in-association sensor-descr (list key "force_per_unit"))))))
     sensor-readings)
    result))

;;;
;;; ROS SETUP TO OBTAIN READINGS
;;;

(defparameter *l-gripper-topic*
  "/pressure/l_gripper_motor/restamped")
(defparameter *r-gripper-topic*
  "/pressure/r_gripper_motor/restamped")

(defparameter *r-gripper-fingertip-subscription* nil)
(defparameter *l-gripper-fingertip-subscription* nil)

(defun subscribe-to-pr2-fingertips ()
  (unless *l-gripper-fingertip-subscription*
    (setf *l-gripper-fingertip-subscription*
          (roslisp:subscribe
           *l-gripper-topic*
           "pr2_msgs/PressureState"
           (lambda (msg)
             (setf (cpl-impl:value *l-gripper-input-fluent*) msg))))))

(defun unsubscribe-pr2-fingertips ()
  (when *l-gripper-fingertip-subscription*
    (roslisp:unsubscribe *l-gripper-fingertip-subscription*)
    (setf *l-gripper-fingertip-subscription* nil)))

(defun pr2-fingertip-msg->map (msg gripper-name)
  (declare (type pr2_msgs-msg:pressurestate)
           (string gripper-name))
  (flet ((get-sensor-associations (finger-name finger-msg)
           (reduce 
            (lambda (assoc-list some-assoc)
              (destructuring-bind (index value) some-assoc
                (cons (conc-strings finger-name "_sensor" (write-to-string index))
                      (cons value assoc-list))))
            (mapcar #'list 
                    (cl-utils:realize (cl-utils:take (length finger-msg) (cl-utils:range)))
                    (coerce finger-msg 'list))
            :initial-value '())))
  (with-fields ((stamp (stamp header)) l_finger_tip r_finger_tip) msg
    (apply #'add-associations
     (make-hash-table :test 'equal)
     (concatenate 'list
                  (get-sensor-associations (conc-strings gripper-name "_l_finger") l_finger_tip)
                  (get-sensor-associations (conc-strings gripper-name "_r_finger") r_finger_tip)
                  (list "stamp" stamp)))
    ;; (make-hash-table-description
    ;;  (conc-strings gripper-name "_l_finger_sensors") 
    ;;  (apply #'make-hash-table-description 
    ;;         (get-sensor-associations (conc-strings gripper-name "_l_finger") l_finger_tip))
    ;;  (conc-strings gripper-name "_r_finger_sensors") 
    ;;  (apply #'make-hash-table-description 
    ;;         (get-sensor-associations (conc-strings gripper-name "_r_finger") r_finger_tip))
    ;;  "stamp" stamp)
    )))

(defun conc-strings (string1 &rest strings)
  (apply #'concatenate 'string string1 strings))

;;;
;;; PRESSURE SENSOR DESCRIPTION PROCESSING
;;;

(defun calc-sensor-normal (halfside1 halfside2 normal-length)
  (cl-transforms:v*
   (cl-transforms::normalize-axis
    (cl-transforms:cross-product halfside1 halfside2))
   normal-length))

(defun calc-sensor-surface-polygons (center halfside1 halfside2)
  (concatenate 
   'list
   (calc-upper-sensor-polygon center halfside1 halfside2)
   (calc-lower-sensor-polygon center halfside1 halfside2)))
               
(defun calc-upper-sensor-polygon (center halfside1 halfside2)
  (list
   (upper-right-corner center halfside1 halfside2)
   (lower-right-corner center halfside1 halfside2)
   (lower-left-corner center halfside1 halfside2)))

(defun calc-lower-sensor-polygon (center halfside1 halfside2)
  (list
   (lower-left-corner center halfside1 halfside2)
   (upper-left-corner center halfside1 halfside2)
   (upper-right-corner center halfside1 halfside2)))

(defun calc-sensor-corner (center halfside1 halfside2 add-1-p add-2-p)
  (let ((fun1 (if add-1-p
                  #'cl-transforms:v+
                  #'cl-transforms:v-))
        (fun2 (if add-2-p
                  #'cl-transforms:v+
                  #'cl-transforms:v-)))
    (funcall fun1 (funcall fun2 center halfside1) halfside2)))

(defun upper-right-corner (center halfside1 halfside2)
  (calc-sensor-corner center halfside1 halfside2 t t))

(defun lower-right-corner (center halfside1 halfside2)
  (calc-sensor-corner center halfside1 halfside2 nil t))

(defun lower-left-corner (center halfside1 halfside2)
  (calc-sensor-corner center halfside1 halfside2 nil nil))

(defun upper-left-corner (center halfside1 halfside2)
  (calc-sensor-corner center halfside1 halfside2 t nil))

;;;
;;; RVIZ VISUALIZATION OF PR2 FINGERTIP
;;; PRESSURE SENSORS DESCRIPTION FROM YAML-FILE
;;;

(defparameter *marker-topic*
  "/visualization_marker")

(defun visualization-publisher (&optional (topic *marker-topic*))
  (roslisp:advertise topic "visualization_msgs/Marker"))

(defun visualize-pr2-gripper-sensor-yaml (directory filename)
  (maphash #'visualize-sensor-description 
           (read-pressure-sensor-descriptions directory filename)))

(defun visualize-sensor-description (sensor-name sensor-descr)
  (roslisp:publish 
   (visualization-publisher)
   (make-sensor-descr-normal-vis-msg sensor-name sensor-descr))
  (roslisp:publish 
   (visualization-publisher)
   (make-sensor-descr-surface-vis-msg sensor-name sensor-descr)))

(defun marker-red ()
  (roslisp:make-message
   "std_msgs/colorrgba"
   :r 1.0
   :a 1.0))

(defun 3d-vector->msg (vec)
  (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z)) vec
    (roslisp:make-message "geometry_msgs/vector3" :x x :y y :z z)))

(defun make-sensor-descr-normal-vis-msg (sensor-name sensor-descr)
  (let* ((normal-length 0.02)
         (marker-width 0.002)
         (marker-arrow-width 0.005)
         (center (get-association-with-error sensor-descr "center"))
         (halfside1 (get-association-with-error sensor-descr "halfside1"))
         (halfside2 (get-association-with-error sensor-descr "halfside2"))
         (frame-id (get-association-with-error sensor-descr "frame_id")))
    (roslisp:make-msg
     "visualization_msgs/Marker"
     (stamp header) (roslisp:ros-time)
     (frame_id header) frame-id
     (ns) sensor-name
     (type) (roslisp-msg-protocol:symbol-code 'visualization_msgs-msg:marker :arrow)
     (action) (roslisp-msg-protocol:symbol-code 'visualization_msgs-msg:marker :add)
     (points) (coerce (mapcar #'3d-vector->msg
                              (list center
                                    (cl-transforms:v+ 
                                     center
                                     (calc-sensor-normal halfside1 halfside2 normal-length))))
                      'vector)
     (color) (marker-red)
     (x scale) marker-width
     (y scale) marker-arrow-width
     (frame_locked) t)))

(defun make-sensor-descr-surface-vis-msg (sensor-name sensor-descr)
  (roslisp:make-msg
   "visualization_msgs/Marker"
   (stamp header) (roslisp:ros-time)
   (frame_id header) (get-association-with-error sensor-descr "frame_id")
   (id) 1
   (ns) sensor-name
   (type) (roslisp-msg-protocol:symbol-code 'visualization_msgs-msg:marker :triangle_list)
   (action) (roslisp-msg-protocol:symbol-code 'visualization_msgs-msg:marker :add)
   (w orientation pose) 1.0
   (points) (coerce 
             (mapcar #'3d-vector->msg
                     (calc-sensor-surface-polygons 
                      (get-association-with-error sensor-descr "center")
                      (get-association-with-error sensor-descr "halfside1")
                      (get-association-with-error sensor-descr "halfside2")))
             'vector)
   (colors) (coerce (loop for i from 0 to 5 collect (marker-red)) 'vector)
   (x scale) 1
   (y scale) 1
   (z scale) 1
   (frame_locked) t))

;;;
;;; READING IN YAML DESCRIPTION OF PR2 FINGERTIP
;;; PRESSURE SENSORS
;;;

(defparameter *pr2-pressure-sensor-description-dir*
  "/home/georg/ros/hydro/catkin_ws/src/iai_robots/iai_pr2_description/sensors")

(defparameter *pr2-pressure-sensor-description-filename*
  "pr2-fingertip-pressure-sensors.yaml")

(defparameter *pr2-fingertip-description* nil)

(defun pr2-fintertip-description ()
  (unless *pr2-fingertip-description*
    (setf *pr2-fingertip-description*
          (read-pressure-sensor-descriptions 
           *pr2-pressure-sensor-description-dir* 
           *pr2-pressure-sensor-description-filename*)))
  *pr2-fingertip-description*)

(defun read-pressure-sensor-descriptions (directory filename)
  (post-process-gripper-descriptions
   (parse-fingertips-yaml directory filename)))

(defun parse-fingertips-yaml (directory filename)
  (yaml:parse (make-pathname :directory directory :name filename)))

(defun post-process-gripper-descriptions (gripper-descrs)
  (apply #'merge-hash-tables
         (mapcar #'post-process-gripper-description
                 (alexandria:hash-table-values gripper-descrs))))

(defun post-process-gripper-description (gripper-descr)
  (apply #'merge-hash-tables 
         (mapcar #'post-process-finger-description (alexandria:hash-table-values gripper-descr))))
     
(defun post-process-finger-description (finger-descr)
  (let ((new-finger-descr (make-hash-table :test 'equal)))
    (maphash (lambda (sensor-key sensor-descr)
               (add-associations new-finger-descr
                                 sensor-key (post-process-sensor-description sensor-descr)))
             finger-descr)
    new-finger-descr))

(defun post-process-sensor-description (sensor-descr)
  (let ((center (get-association sensor-descr "center"))
        (halfside1 (get-association sensor-descr "halfside1"))
        (halfside2 (get-association sensor-descr "halfside2")))
    (if (and center halfside1 halfside2)
        (add-associations sensor-descr 
                          "center" (post-process-point-descr center)
                          "halfside1" (post-process-point-descr halfside1)
                          "halfside2" (post-process-point-descr halfside2))
        (progn
          (warn "Sensor description has missing keys. Not post processing.")
          sensor-descr))))

(defun post-process-point-descr (point-descr)
  (let ((x (get-association point-descr "x"))
        (y (get-association point-descr "y"))
        (z (get-association point-descr "z")))
    (if (and x y z)
        (cl-transforms:make-3d-vector x y z)
        (progn
          (warn "Point description has missing keys. Not transforming.")
          point-descr))))