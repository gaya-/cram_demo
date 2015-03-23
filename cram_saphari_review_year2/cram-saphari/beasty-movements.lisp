;;; Copyright (c) 2015, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :cram-saphari)

;;;
;;; DEFAULT VALUES
;;;

(defparameter *joint-goal-defaults*
  `(:joint0 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7)
     :joint1 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7)
     :joint2 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7)
     :joint3 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7)
     :joint4 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7)
     :joint5 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7)
     :joint6 (:stiffness 80
              :damping 0.7
              :max-vel 0.35
              :max-acc 0.7)))

(defparameter *cartesian-goal-defaults*
  `(:trans-x (:stiffness 500
              :damping 0.7
              :max-vel 0.6
              :max-acc 0.3)
    :trans-y (:stiffness 500
              :damping 0.7
              :max-vel 0.6
              :max-acc 0.3)
    :trans-z (:stiffness 500
              :damping 0.7
              :max-vel 0.6
              :max-acc 0.3)
    :rot-x (:stiffness 50
            :damping 0.7
            :max-vel 0.7
            :max-acc 1.4)
    :rot-y (:stiffness 50
            :damping 0.7
            :max-vel 0.7
            :max-acc 1.4)
    :rot-z (:stiffness 50
            :damping 0.7
            :max-vel 0.7
            :max-acc 1.4)))

(defparameter *robot-setup-defaults*
  `(:ee-transform ,(cl-transforms:make-transform
                    (cl-transforms:make-3d-vector 0 0 0.319)
                    (cl-transforms:make-identity-rotation))
    :base-transform ,(cl-transforms:make-identity-transform)
    :base-acceleration ,(cl-transforms:make-wrench
                         (cl-transforms:make-3d-vector -7.358 4.248 4.905)
                         (cl-transforms:make-identity-vector))
    :tool-mass 2.5
    :tool-com ,(cl-transforms:make-3d-vector 0.0 0.0 0.19)))

(defparameter *default-safety-strategies*
  `(:contact-strategies ,roslisp-beasty:*default-safety-strategies*))

;;;
;;; AUX CREATION FUNCTIONS
;;;

(defun make-init-description (&optional (sim-p t))
  (concatenate 
   'list
   *joint-goal-defaults*
   *cartesian-goal-defaults*
   *robot-setup-defaults*
   *default-safety-strategies*
   (list :simulated-robot sim-p
         :command-type :initialize)))

(defun make-joint-goal (goal-config &optional (sim-p t))
  (concatenate 
   'list
   *joint-goal-defaults*
   *robot-setup-defaults*
   *default-safety-strategies*
   (list :simulated-robot sim-p
         :command-type :joint-impedance
         :joint-goal-config goal-config)))

(defun make-cartesian-goal (goal-transform &optional (sim-p t))
  (concatenate 
   'list
   *cartesian-goal-defaults*
   *robot-setup-defaults*
   *default-safety-strategies*
   (list :simulated-robot sim-p
         :command-type :cartesian-impedance
         :cartesian-goal-pose (ensure-transform goal-transform))))

(defun ensure-transform (transform)
  (etypecase transform
    (cl-transforms:transform transform)
    (cl-tf2::stamped-transform (cl-tf2::transform transform))))
;;;
;;; VARIOUS AUX
;;; 

(defun collision-p (collision-type)
  (and (roslisp-beasty::collision-type-p collision-type)
       (not (eql :NO-COLLISION collision-type))))

;; (defun plist-alist-recursively (plist)
;;   "Returns an alist containing the keys and values of the property list
;;  `plist'. Recursively calls itself on any of the values of `plist' that are
;;  also property lists. Note: Leaves `plist' untouched."
;;   (flet ((plist-p (obj)
;;            (and (listp obj) (evenp (length obj)) (<= 2 (length obj)))))
;;   (let ((tmp-list nil))
;;     (alexandria:doplist (key value plist)
;;       (setf (getf tmp-list key)
;;             (if (plist-p value)
;;                 (progn
;;                   (list (plist-alist-recursively value))
;;                   (format t "~a~%" value))
;;                 (list value))))
;;     (alexandria:plist-alist tmp-list))))

(defun plist->cram-list-recursively (plist)
  (flet ((plist-p (obj)
           (and (listp obj) (evenp (length obj)) (<= 2 (length obj)))))
    (let ((result nil))
      (alexandria:doplist (key value plist)
        (push (list key (if (plist-p value)
                            (plist->cram-list-recursively value)
                            value))
              result))
      result)))
    
;;;
;;; ACTION DESIGNATOR RESOLUTION
;;;

(defun broadcast-transform (transform &optional (handle *handle*))
  (cl-tf2:send-transform (tf-broadcaster handle) transform))

(def-fact-group beasty-action-designators (action-desig)

  (<- (beasty-desig? ?desig)
    (action-desig? ?desig)
    (desig-prop ?desig (:controller :beasty)))

  (<- (right-beasty-desig? ?desig)
    (beasty-desig? ?desig)
    (desig-prop ?desig (:arm :right)))

  (<- (action-desig ?desig ?beasty-goal)
    (right-beasty-desig? ?desig)
    (desig-prop ?desig (:field-of-view :clear))
    (lisp-fun make-joint-goal (0.52 -0.78 0.78 0.78 0.0 -0.78 0.0) nil ?beasty-goal)
    (lisp-fun add-motion-command-to-desig ?desig ?beasty-goal ?_))

  (<- (action-desig ?desig ?beasty-goal)
    (right-beasty-desig? ?desig)
    (desig-prop ?desig (:move :over))
    (desig-prop ?desig (:object ?object))
    (lisp-fun desig->pregrasp-transform ?object -0.1 ?transform)
    (lisp-fun broadcast-transform ?transform ?_)
    (lisp-fun make-cartesian-goal ?transform nil ?beasty-goal)
    (lisp-fun add-motion-command-to-desig ?desig ?beasty-goal ?_))

  (<- (action-desig ?desig ?beasty-goal)
    (right-beasty-desig? ?desig)
    (desig-prop ?desig (:move :down))
    (lisp-fun relative-cartesian-transform 0.15 ?transform)
    (lisp-fun broadcast-transform ?transform ?_)
    (lisp-fun make-cartesian-goal ?transform nil ?beasty-goal)
    (lisp-fun add-motion-command-to-desig ?desig ?beasty-goal ?_))
;;; (symbol-function 'move-cartesian)
)

(defun add-motion-command-to-desig (desig goal)
  (let ((new-goal (copy-list goal)))
    ;; HACK for logging: lists of values should be vectors
    (setf (getf new-goal :joint-goal-config)
          (coerce (getf new-goal :joint-goal-config) 'vector))
    (let ((new-desig (copy-designator desig :new-description (plist->cram-list-recursively new-goal))))
      ;; manually setting the solutions slot of the new desig to avoid re-resolution
      (setf (slot-value new-desig 'desig::solutions) (list goal))
      (equate desig new-desig))))
             
(defun desig->stamped-transform (desig)
  (let ((pose (desig-prop-value desig 'desig-props:pose))
        (name (desig-prop-value desig 'desig-props::response)))
    (with-slots ((frame-id cl-tf-datatypes:frame-id) (stamp cl-tf-datatypes:stamp) 
                 (origin cl-transforms:origin) (orientation cl-transforms:orientation)) pose
      (cl-tf2:make-stamped-transform
       frame-id name stamp
       (cl-transforms:make-transform origin orientation)))))

(defun desig->pregrasp-transform (desig offset &optional (handle *handle*))
  (let ((tool-transform (desig->stamped-transform desig))
        (offset (cl-transforms:make-transform
                 (cl-transforms:make-3d-vector 0 0 offset)
                 (cl-transforms:make-identity-rotation)))
        (hand-eye (tf2-lookup
                   (tf handle) 
                   
                   "calib_right_arm_base_link"
                   "head_mount_kinect2_rgb_optical_frame"))
        (hand-rotation
          (cl-transforms:make-transform
           (cl-transforms:make-identity-vector)
           (cl-transforms:axis-angle->quaternion 
            (cl-transforms:make-3d-vector 0 0 1)
            (/ PI 2.0))))
)
    (with-slots ((transform cl-tf2::transform) (header cl-tf2::header)
                 (child-frame-id cl-tf2::child-frame-id)) tool-transform
      (cl-tf2:make-stamped-transform
       "calib_right_arm_base_link"
       (concatenate 'string child-frame-id "_pregrasp") 
       (roslisp:ros-time) 
       (cl-transforms:transform* (cl-tf2::transform hand-eye) transform offset hand-rotation)
       ))))
        
(defun tf2-lookup (tf frame-id child-frame-id)
  (handler-case (cl-tf2:lookup-transform tf frame-id child-frame-id)
    (cl-tf2::tf2-server-error () (progn (cpl:sleep 0.1) (tf2-lookup tf frame-id child-frame-id)))))

(defun relative-cartesian-transform (z-offset &optional (handle *handle*))
  (let ((hand-transform 
          (tf2-lookup (tf handle) "calib_right_arm_base_link" "right_gripper_tool_frame"))
        (offset (cl-transforms:make-transform
                 (cl-transforms:make-3d-vector 0 0 z-offset)
                 (cl-transforms:make-identity-rotation))))
    (with-slots ((transform cl-tf2::transform) (header cl-tf2::header)
                 (child-frame-id cl-tf2::child-frame-id)) hand-transform
      (cl-tf2:make-stamped-transform
       "calib_right_arm_base_link"
       (concatenate 'string child-frame-id "_downgoal") 
       (roslisp:ros-time) 
       (cl-transforms:transform* (cl-tf2::transform hand-transform) offset)))))

;;;
;;; LOGGING HOOKS
;;;
 
(defun begin-logging-safety-reset (action-desig)
  (let ((id (beliefstate:start-node "SAFETY-RESET" nil 2)))
    (beliefstate:add-designator-to-node action-desig id)
    id))

(defun finish-logging-safety-reset (logging-id)
  (beliefstate:stop-node logging-id))
      
(defun log-collision (collision)
  (let ((logging-id (beliefstate:start-node (string collision) nil 2)))
    (beliefstate:stop-node logging-id)))

;;;
;;; CRAM PLAN INTERFACE
;;;

(cpl-impl:def-cram-function safely-perform-action (arm action)
  (cpl-impl:pursue
    (cpl-impl:on-suspension (roslisp-beasty:stop-beasty arm)
      (cpl-impl:retry-after-suspension
        (when (collision-p (value (roslisp-beasty:collision-fluent arm)))
          (reset-safety arm action))
        (roslisp-beasty:move-beasty-and-wait arm (reference action))))
    (whenever ((pulsed (roslisp-beasty:collision-fluent arm)))
      (when (collision-p (value (roslisp-beasty:collision-fluent arm)))
        (log-collision (value (roslisp-beasty:collision-fluent arm)))))))
          
(cpl-impl:def-cram-function reset-safety (arm action)
  (roslisp-beasty:beasty-safety-reset arm (reference action))
  (roslisp-beasty:beasty-switch-behavior arm (reference action)))