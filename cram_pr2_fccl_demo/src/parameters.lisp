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

(in-package :pr2-fccl-demo)

;;;
;;; AUX METHODS TO INIT PARAMETERS
;;;

(defun make-joint-state-list (joint-names joint-positions)
  "Takes a list of `joint-names` and a list of `joint-positions`, and
 returns a list joint-states. Input lists need to be of equal length."
  (mapcar (lambda (name position)
            (cl-robot-models:make-joint-state :name name :position position))
          joint-names joint-positions))

(defun mirror-prefix (prefix)
  (cond
    ((string= "l" prefix) "r")
    ((string= "r" prefix) "l")))

(defun mirror-joint-name (joint-name)
  (concatenate 'string 
               (mirror-prefix (subseq joint-name 0 1))
               (subseq joint-name 1 (length joint-name))))

(defun mirrored-joint-position-p (joint-name)
  (find joint-name *mirrored-joints* :test #'string=))

(defun mirror-joint-position (joint-name joint-position)
  (if (mirrored-joint-position-p joint-name)
      (- joint-position)
      joint-position))

(defun mirror-joint-state (joint-state)
  (let ((new-joint-state (cl-robot-models:copy-joint-state joint-state)))
    (with-slots ((joint-name cl-robot-models::name)
                 (joint-position cl-robot-models::position)) new-joint-state
      (setf joint-position (mirror-joint-position joint-name joint-position))
      (setf joint-name (mirror-joint-name joint-name))
    new-joint-state)))

(defun mirror-config (config)
  (with-slots ((joint-states cl-robot-models:joint-states)
               (robot-model cl-robot-models:robot-model)
               (robot-name cl-robot-models:robot-name)) config
    (cl-robot-models:make-robot-state 
     robot-name robot-model 
    (mapcar #'mirror-joint-state (alexandria:hash-table-values joint-states)))))

;;;
;;; KINEMATICS OF LEFT AND RIGHT ARM
;;;

(defparameter *l-arm-base-link* "torso_lift_link")
(defparameter *l-arm-tip-link* "l_gripper_tool_frame")
(defparameter *l-arm-kinematic-chain* 
  (cram-fccl:make-kinematic-chain *l-arm-base-link* *l-arm-tip-link*))

(defparameter *r-arm-base-link* "torso_lift_link")
(defparameter *r-arm-tip-link* "r_gripper_tool_frame")
(defparameter *r-arm-kinematic-chain* 
  (cram-fccl:make-kinematic-chain *r-arm-base-link* *r-arm-tip-link*))

(defparameter *l-arm-joint-names*
  '("l_upper_arm_roll_joint"
    "l_shoulder_pan_joint"
    "l_shoulder_lift_joint"
    "l_forearm_roll_joint"
    "l_elbow_flex_joint"
    "l_wrist_flex_joint"
    "l_wrist_roll_joint"))

(defparameter *r-arm-joint-names*
  (mapcar #'mirror-joint-name *l-arm-joint-names*))

(defparameter *r-arm-mirrored-joints*
  '("r_upper_arm_roll_joint"
    "r_shoulder_pan_joint"
    "r_forearm_roll_joint"
    "r_wrist_roll_joint"))

(defparameter *l-arm-mirrored-joints*
  (mapcar #'mirror-joint-name *r-arm-mirrored-joints*))

(defparameter *mirrored-joints*
  (concatenate 'list 
               *r-arm-mirrored-joints* 
               *l-arm-mirrored-joints*))

(defun other-arm (arm)
  (ecase arm
    (right-arm 'left-arm)
    (left-arm 'right-arm)))

;;;
;;; RESTING CONFIGURATIONS FOR ARMS
;;;

(defparameter *r-arm-resting-config* 
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (make-joint-state-list
    *r-arm-joint-names*
    '(-1.964 -1.265 1.267 5.82 -0.263 -0.132 2.641))))

(defparameter *l-arm-resting-config*
  (mirror-config *r-arm-resting-config*)
  ;; (cl-robot-models:make-robot-state
  ;;  "Raphael" "PR2"
  ;;  (make-joint-state-list
  ;;   *l-arm-joint-names*
  ;;   '(1.964 1.265 1.267 -5.82 -0.263 -0.132 -2.641)))
  )

(defun resting-config (arm)
  (ecase arm
    (right-arm *r-arm-resting-config*)
    (left-arm *l-arm-resting-config*)))

;;;
;;; START CONFIGURATIONS FOR POURING
;;;

(defparameter *r-arm-pouring-start-config*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (make-joint-state-list
    *r-arm-joint-names*
    '(-1.393 -1.065 0.264 -0.524 -1.63 -0.967 1.861))))

(defparameter *l-arm-pouring-start-config*
  ;; (cl-robot-models:make-robot-state
  ;;  "Raphael" "PR2"
  ;;  (make-joint-state-list
  ;;   *l-arm-joint-names*
  ;;   '(1.964 1.265 1.267 -5.82 -0.263 -0.132 -2.64)))
  (mirror-config *r-arm-pouring-start-config*))

(defun pouring-config (arm)
  (ecase arm
    (right-arm *r-arm-pouring-start-config*)
    (left-arm *l-arm-pouring-start-config*)))

(defun get-pouring-start-config (pouring-arm query-arm)
  (if (eql pouring-arm query-arm)
      (pouring-config query-arm)
      (resting-config query-arm)))

;;;
;;; START CONFIGURATIONS FOR FLIPPING
;;;

(defparameter *l-arm-flipping-start-config*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (make-joint-state-list
    *l-arm-joint-names*
    '(1.32 1.08 0.16 0.0 -1.14 -1.05 1.57))))

(defparameter *r-arm-flipping-start-config*
  ;; (cl-robot-models:make-robot-state
  ;;  "Raphael" "PR2"
  ;;  (make-joint-state-list
  ;;   *r-arm-joint-names*
  ;;   '(-1.32 -1.08 0.16 0.0 -1.14 -1.05 1.57)))
  (mirror-config *l-arm-flipping-start-config*))

(defun flipping-config (arm)
  (ecase arm
    (right-arm *r-arm-flipping-start-config*)
    (left-arm *l-arm-flipping-start-config*)))

;;;
;;; START CONFIGURATIONS FOR GRASPING
;;;

(defparameter *l-arm-grasping-configuration*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (make-joint-state-list
    *l-arm-joint-names*
    '(0.593 1.265 0.964 0.524 -2.1 -0.067 4.419))))

(defparameter *r-arm-grasping-configuration*
  (resting-config 'right-arm))

;;;
;;; STANDARD POSITION CONTROLLERS FOR PR2 ARMS
;;;

(defparameter *l-arm-position-controller-action-name* 
  "/l_arm_controller/joint_trajectory_action")

(defparameter *r-arm-position-controller-action-name* 
  "/r_arm_controller/joint_trajectory_action")

;;;
;;; FCCL CONTROLLER FOR LEFT ARM
;;;

(defparameter *l-arm-fccl-controller-action-name* "/l_arm_fccl_controller/command")
(defparameter *r-arm-fccl-controller-action-name* "/r_arm_fccl_controller/command")

;;;
;;; TF RELAY TOPIC
;;;

(defparameter *tf-relay-topic* "/tf_relay/in_topic")