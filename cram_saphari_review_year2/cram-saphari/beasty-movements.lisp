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

;;;
;;; AUX CREATION FUNCTIONS
;;;

(defun make-init-description (&optional (sim-p t))
  (concatenate 
   'list
   *joint-goal-defaults*
   *cartesian-goal-defaults*
   *robot-setup-defaults*
   (list :simulated-robot sim-p
         :command-type :initialize)))

(defun make-joint-goal (goal-config &optional (sim-p t))
  (concatenate 
   'list
   *joint-goal-defaults*
   *robot-setup-defaults*
   (list :simulated-robot sim-p
         :command-type :joint-impedance
         :joint-goal-config goal-config)))

(defun make-cartesian-goal (goal-transform &optional (sim-p t))
  (concatenate 
   'list
   *cartesian-goal-defaults*
   *robot-setup-defaults*
   (list :simulated-robot sim-p
         :command-type :cartesian-impedance
         :cartesian-goal-pose goal-transform)))