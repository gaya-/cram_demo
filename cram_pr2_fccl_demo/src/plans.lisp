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

(cpl-impl:def-cram-function pancake-pouring-test ()
  (let ((pancake-mix-pose
          (cl-tf:make-pose-stamped
           "r_gripper_tool_frame" 0
           (cl-transforms:make-3d-vector 0.0 0.0 0.0)
           (cl-transforms:make-identity-rotation)))
        (pancake-maker-pose
          (cl-tf:make-pose-stamped
           "map" 0
           (cl-transforms:make-3d-vector 0.434 0.0 0.703)
           (cl-transforms:make-identity-rotation))))
    (with-designators ((pancake-mix (cram-designators:object '()))
                       (pancake-maker (cram-designators:object '())))
      (set-pose pancake-mix pancake-mix-pose)
      (set-pose pancake-maker pancake-maker-pose)
      (demo-part-pouring pancake-mix pancake-maker))))

(cpl-impl:def-cram-function pancake-flipping-test ()
  (let ((l-spatula-pose
          (cl-tf:make-pose-stamped
           "l_gripper_tool_frame" 0
           (cl-transforms:make-3d-vector 0.0 0.0 0.0)
           (cl-transforms:make-identity-rotation)))
        (r-spatula-pose
          (cl-tf:make-pose-stamped
           "r_gripper_tool_frame" 0
           (cl-transforms:make-3d-vector 0.0 0.0 0.0)
           (cl-transforms:make-identity-rotation)))
        (pancake-pose
          (cl-tf:make-pose-stamped
           "map" 0
           (cl-transforms:make-3d-vector 0.434 0.0 0.753)
           (cl-transforms:make-identity-rotation)))
        (pancake-maker-pose
          (cl-tf:make-pose-stamped
           "map" 0
           (cl-transforms:make-3d-vector 0.434 0.0 0.703)
           (cl-transforms:make-identity-rotation))))
    (with-designators ((l-spatula (cram-designators:object '()))
                       (r-spatula (cram-designators:object '()))
                       (pancake (cram-designators:object '()))
                       (pancake-maker (cram-designators:object '())))
      (set-pose l-spatula l-spatula-pose)
      (set-pose r-spatula r-spatula-pose)
      (set-pose pancake pancake-pose)
      (set-pose pancake-maker pancake-maker-pose)
      (demo-part-flipping l-spatula r-spatula pancake pancake-maker))))
          
(defun set-pose (obj-desig pose)
  (with-slots (cram-designators:data) obj-desig
    (setf cram-designators:data pose)))

(defun get-transform (obj-desig child-frame-id)
  (with-slots (cl-tf:frame-id cl-tf:stamp cl-tf:origin cl-tf:orientation)
      (reference obj-desig)
    (cl-tf:make-stamped-transform
     cl-tf:frame-id child-frame-id cl-tf:stamp cl-tf:origin cl-tf:orientation)))

(cpl-impl:def-cram-function demo-part-pouring (pancake-mix pancake-maker)
  (with-designators ((desig (action `((type constraints) (to pour)))))
    (destructuring-bind (motions start-controller stop-controller finished-fluent)
        (reference desig)
      (cl-tf::with-tf-broadcasting ((get-tf-broadcaster) 
                                    (get-transform pancake-mix "pancake_bottle")
                                    (get-transform pancake-maker "pancake_maker"))
        (cpl-impl:sleep* 2)
        (ensure-vel-controllers)
        (loop for motion in motions do
          (add-motion-phase-to-designator-description desig (id motion))
          (cram-language:pursue
            (funcall start-controller motion)
            (cram-language:whenever ((cram-language:pulsed finished-fluent))
              (when (cram-language-implementation:value finished-fluent)
                (funcall stop-controller)))))
        (ensure-pos-controllers)
        (format t "~a~%" (cram-designators:current-desig desig))))))

(cpl-impl:def-cram-function demo-part-flipping 
    (spatula-left spatula-right pancake pancake-maker)
  (with-designators ((desig (action `((type constraints) (to flip)))))
  (destructuring-bind (motions l-start-controller l-stop-controller l-finished-fluent
                       r-start-controller r-stop-controller r-finished-fluent)
      (reference desig)
    (cl-tf::with-tf-broadcasting 
        ((get-tf-broadcaster)
         (get-transform pancake "pancake")
         (get-transform spatula-left "l_spatula_handle")
         (get-transform spatula-right "r_spatula_handle")
         (get-transform pancake-maker "pancake_maker"))
      (cpl-impl:sleep* 2)
      (ensure-vel-controllers)
      (loop for motion in motions do
        (cram-language:pursue
          (funcall l-start-controller (first motion))
          (funcall r-start-controller (rest motion))
          (cram-language:whenever ((cpl-impl:fl-or 
                                    (cram-language:pulsed l-finished-fluent)
                                    (cram-language:pulsed r-finished-fluent)))
            (when (cpl-impl:fl-and (cram-language-implementation:value l-finished-fluent)
                                   (cram-language-implementation:value r-finished-fluent))
              (cram-language:par
                (funcall l-stop-controller)
                (funcall r-stop-controller))))))
      (ensure-pos-controllers)))))

(cpl-impl:def-cram-function move-into-flipping-configuration ()
  (ensure-pos-controllers)
  (cram-language:par
    (pr2-controllers:move-arm (get-right-arm-position-controller)
                              *r-arm-flipping-start-config* 4.0)
    (pr2-controllers:move-arm (get-left-arm-position-controller)
                              *l-arm-flipping-start-config* 4.0)))

(cpl-impl:def-cram-function move-into-pouring-configuration ()
  (ensure-pos-controllers)
  (cram-language:par
    (pr2-controllers:move-arm (get-right-arm-position-controller)
                              *r-arm-pouring-start-config* 4.0)
    (pr2-controllers:move-arm (get-left-arm-position-controller)
                              *l-arm-pouring-start-config* 4.0)))

(cpl-impl:def-cram-function move-into-grasping-configuration ()
  (ensure-pos-controllers)
  (cram-language:par
    (pr2-controllers:move-arm 
     (get-right-arm-position-controller)
     *r-arm-grasping-configuration* 4.0)
    (pr2-controllers:move-arm
     (get-left-arm-position-controller)
     *l-arm-grasping-configuration* 4.0)))

(defun add-motion-phase-to-designator-description (old-desig new-phase-id)
  (let ((current-desig (cram-designators:current-desig old-desig)))
    (cram-designators:with-desig-props (phases) current-desig
    (cram-designators:equate
     current-desig
     (cram-designators:copy-designator
      current-desig
      :new-description (generate-new-motion-phase-description phases new-phase-id))))))

(defun generate-new-motion-phase-description (old-phase-description new-phase-id)
  (if old-phase-description
      `((phases ,(append old-phase-description `((phase ,new-phase-id)))))
      `((phases ((phase ,new-phase-id))))))