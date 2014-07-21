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
  '("r_upper_arm_roll_joint"
    "r_shoulder_pan_joint"
    "r_shoulder_lift_joint"
    "r_forearm_roll_joint"
    "r_elbow_flex_joint"
    "r_wrist_flex_joint"
    "r_wrist_roll_joint"))

;;;
;;; START CONFIGURATIONS FOR LEFT ARM
;;;

(defparameter *r-arm-pouring-start-config*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (list
    (cl-robot-models:make-joint-state
     :name "r_upper_arm_roll_joint" :position -1.392565491097796)
    (cl-robot-models:make-joint-state
     :name "r_shoulder_pan_joint" :position -1.0650093105988152)
    (cl-robot-models:make-joint-state
     :name "r_shoulder_lift_joint" :position 0.26376743371555295)
    (cl-robot-models:make-joint-state
     :name "r_forearm_roll_joint" :position -0.524)
    (cl-robot-models:make-joint-state
     :name "r_elbow_flex_joint" :position -1.629946646305397)
    (cl-robot-models:make-joint-state
     :name "r_wrist_flex_joint" :position -0.9668414952685922)
    (cl-robot-models:make-joint-state
     :name "r_wrist_roll_joint" :position 1.8614))))

(defparameter *l-arm-pouring-start-config*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (list
    (cl-robot-models:make-joint-state
     :name "l_upper_arm_roll_joint" :position 1.9643297630604963)
    (cl-robot-models:make-joint-state
     :name "l_shoulder_pan_joint" :position 1.265335905500992)
    (cl-robot-models:make-joint-state
     :name "l_shoulder_lift_joint" :position 1.2666995326579538)
    (cl-robot-models:make-joint-state
     :name "l_forearm_roll_joint" :position -5.81991983730232)
    (cl-robot-models:make-joint-state
     :name "l_elbow_flex_joint" :position -0.2625872772879775)
    (cl-robot-models:make-joint-state
     :name "l_wrist_flex_joint" :position -0.13242260444085052)
    (cl-robot-models:make-joint-state
     :name "l_wrist_roll_joint" :position -2.64))))

(defparameter *l-arm-flipping-start-config*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"

   (list
    (cl-robot-models:make-joint-state
     :name "l_upper_arm_roll_joint" :position 1.32)
    (cl-robot-models:make-joint-state
     :name "l_shoulder_pan_joint" :position 1.08)
    (cl-robot-models:make-joint-state
     :name "l_shoulder_lift_joint" :position 0.16)
    (cl-robot-models:make-joint-state
     :name "l_forearm_roll_joint" :position 0.0)
    (cl-robot-models:make-joint-state
     :name "l_elbow_flex_joint" :position -1.14)
    (cl-robot-models:make-joint-state
     :name "l_wrist_flex_joint" :position -1.05)
    (cl-robot-models:make-joint-state
     :name "l_wrist_roll_joint" :position 1.57))))

(defparameter *r-arm-flipping-start-config*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (list
    (cl-robot-models:make-joint-state
     :name "r_upper_arm_roll_joint" :position -1.32)
    (cl-robot-models:make-joint-state
     :name "r_shoulder_pan_joint" :position -1.08)
    (cl-robot-models:make-joint-state
     :name "r_shoulder_lift_joint" :position 0.16)
    (cl-robot-models:make-joint-state
     :name "r_forearm_roll_joint" :position 0.0)
    (cl-robot-models:make-joint-state
     :name "r_elbow_flex_joint" :position -1.14)
    (cl-robot-models:make-joint-state
     :name "r_wrist_flex_joint" :position -1.05)
    (cl-robot-models:make-joint-state
     :name "r_wrist_roll_joint" :position 1.57))))

(defparameter *r-arm-grasping-configuration*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (make-joint-state-list
    *r-arm-joint-names*
    '(-1.964 -1.265 1.267 5.82 -0.263 -0.132 2.641))))

(defparameter *l-arm-grasping-configuration*
  (cl-robot-models:make-robot-state
   "Raphael" "PR2"
   (make-joint-state-list
    *l-arm-joint-names*
    '(0.593 1.265 0.964 0.524 -2.1 -0.067 4.419))))

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