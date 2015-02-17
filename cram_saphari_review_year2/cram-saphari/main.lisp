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



(defclass cram-saphari-demo ()
  ((human-percept :initarg :human-percept :accessor human-percept)
   (human-sub :initarg :human-sub :accessor human-sub)
   (right-arm :initarg :right-arm :accessor right-arm)))

(defun init-cram-saphari-demo ()
  (let* ((human-percept (make-fluent))
         (human-sub (subscribe "/saphari/human" "saphari_msgs/Human" 
                               (lambda (msg) (setf (value human-percept) (from-msg msg)))))
         (right-arm (roslisp-beasty:make-beasty-handle "right_arm" 1 1337)))
    (roslisp-beasty:beasty-switch-behavior right-arm (make-init-description nil))
    (roslisp-beasty:beasty-safety-reset right-arm (make-init-description nil))
    (make-instance 
     'cram-saphari-demo
     :human-percept human-percept
     :human-sub human-sub
     :right-arm right-arm)))
   
(cpl-impl:def-top-level-cram-function test-suspend-motions (demo-handle)
  (with-slots (human-percept right-arm) demo-handle
    (with-human-monitoring (human-percept) 
      (cpl-impl:on-suspension (roslisp-beasty:stop-beasty right-arm)
        (cpl-impl:retry-after-suspension
          (roslisp-beasty:move-beasty-and-wait 
           right-arm (make-joint-goal '(0.52 -0.78 0.78 0.78 0.0 -0.78 0.0) nil)))))
    (with-human-monitoring (human-percept) 
      (cpl-impl:on-suspension (roslisp-beasty:stop-beasty right-arm)
        (cpl-impl:retry-after-suspension
          (roslisp-beasty:move-beasty-and-wait 
           right-arm 
           (make-cartesian-goal 
            (cl-transforms:make-transform
             (cl-transforms:make-3d-vector 0.359 0.459 0.533)
             (cl-transforms:axis-angle->quaternion
              (cl-transforms:make-3d-vector 0 1 0) PI)) nil)))))))

(defun main ()
  (with-ros-node ("saphari_demo")
    (test-suspend-motions (init-cram-saphari-demo))))

(defun dump-files ()
  (with-ros-node ("saphari_demo")
    (beliefstate:extract-files)))