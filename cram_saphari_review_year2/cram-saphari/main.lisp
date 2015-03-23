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
   (right-arm :initarg :right-arm :accessor right-arm)
   (tf-broadcaster :initarg :tf-broadcaster :accessor tf-broadcaster)
   (tf :initarg :tf :accessor tf)
   (wsg50 :initarg :wsg50 :accessor wsg50)))

(defun init-cram-saphari-demo ()
  (let* ((human-percept (make-fluent))
         (human-sub (subscribe "/saphari/human" "saphari_msgs/Human" 
                               (lambda (msg) (setf (value human-percept) (from-msg msg)))))
         (right-arm (roslisp-beasty:make-beasty-handle "right_arm" 1 1337))
         (tf-broadcaster (cl-tf2:make-transform-broadcaster))
         (tf (make-instance 'cl-tf2:buffer-client))
         (wsg50 (cram-wsg50:make-wsg50-handle "wsg_50_driver")))
    (cram-uima:init-uima-bridge)
    (roslisp-beasty:beasty-switch-behavior right-arm (make-init-description nil))
    (roslisp-beasty:beasty-safety-reset right-arm (make-init-description nil))
    (make-instance 
     'cram-saphari-demo
     :human-percept human-percept
     :human-sub human-sub
     :right-arm right-arm
     :tf-broadcaster tf-broadcaster
     :tf tf
     :wsg50 wsg50)))
   
(defun on-prepare-perception-request (designator-request)
  (let ((id (beliefstate:start-node
             "UIMA-PERCEIVE"
             (cram-designators:description designator-request) 2)))
    (beliefstate:add-designator-to-node designator-request id :annotation "perception-request")
    id))

(defun on-finish-perception-request (id designators-result)
  (dolist (desig designators-result)
    (beliefstate:add-designator-to-node
     desig id :annotation "perception-result"))
;  (beliefstate:add-topic-image-to-active-node cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node id :success (not (eql designators-result nil))))

(defparameter *non-instrument-objects* (list "bowl" "TablePlane"))

(defun filter-out-non-instrument-desigs (desigs)
  (remove-if (lambda (name) (member name *non-instrument-objects* :test #'string=))
             desigs :key (rcurry #'desig-prop-value 'desig-props::response)))

(defun find-bowl (desigs)
  (find "bowl" desigs 
        :key (rcurry #'desig-prop-value 'desig-props::response) 
        :test #'string=))

(cpl-impl:def-top-level-cram-function saphari-main (demo-handle)
  (loop do (grasp-any-unsorted-object demo-handle)))

(cpl-impl:def-cram-function move-arm-away (demo-handle)
  (with-slots (human-percept right-arm) demo-handle
    (cpl-desig-supp:with-designators 
        ((arm-away-action (action '((:controller :beasty)
                                    (:arm :right)
                                    (:field-of-view :clear)))))
      (with-human-monitoring (human-percept)
        (safely-perform-action right-arm arm-away-action))
      arm-away-action)))

(cpl-impl:def-cram-function display-objects (handle)
  (apply #'send-transform (tf-broadcaster handle)
         (mapcar (rcurry #'desig->pregrasp-transform -0.1 handle) (perceive-objects))))
    
(cpl-impl:def-cram-function perceive-objects ()
  (cpl-desig-supp:with-designators ((obj-desig (object nil)))
    (let* ((logging-id (on-prepare-perception-request obj-desig))
           (desigs (cram-uima:get-uima-result obj-desig)))
      (on-finish-perception-request logging-id desigs)
      desigs)))

(cpl-impl:def-cram-function perceive-instrument-and-bowl ()
  (let ((desigs (perceive-objects)))
    (values 
     (first (filter-out-non-instrument-desigs desigs)) (find-bowl desigs))))
 
(cpl-impl:def-cram-function grasp-any-unsorted-object (handle)
  (move-arm-away handle)
  (multiple-value-bind (instrument bowl) (perceive-instrument-and-bowl)
    (when (and instrument bowl)
      (move-over-object instrument handle)
      (cram-wsg50:move-wsg50 (wsg50 handle) 90 100 10)
      (move-down-until-touch handle)
      (cram-wsg50:move-wsg50 (wsg50 handle) 0 100 50)
      (cpl:sleep 1)
      (move-over-object instrument handle)
      (move-over-object bowl handle)
      (cram-wsg50:move-wsg50 (wsg50 handle) 90 100 10)

)))

(cpl-impl:def-cram-function move-over-object (object handle)
  (with-slots (human-percept right-arm) handle
      (cpl-desig-supp:with-designators 
          ((pregrasp (action `((:controller :beasty) (:arm :right)
                               (:move :over) (:object ,object)))))
        (with-human-monitoring (human-percept)
          (safely-perform-action right-arm pregrasp)))))

(cpl-impl:def-cram-function move-down-until-touch (handle)
  (with-slots (human-percept right-arm) handle
    (cpl:pursue
      (loop do
        (cpl-desig-supp:with-designators 
            ((pregrasp (action `((:controller :beasty) (:arm :right) (:move :down)))))
          (with-human-monitoring (human-percept)
            (safely-perform-action right-arm pregrasp))))
      (cpl-impl:wait-for (cpl:not (cpl:eql 
                                   (roslisp-beasty:collision-fluent right-arm) :NO-COLLISION))))))

                 
;; (cpl-impl:def-top-level-cram-function test-suspend-motions (demo-handle)
;;   (with-slots (human-percept right-arm) demo-handle
;;     (with-human-monitoring (human-percept) 
;;       (cpl-impl:on-suspension (roslisp-beasty:stop-beasty right-arm)
;;         (cpl-impl:retry-after-suspension
;;           (roslisp-beasty:beasty-safety-reset
;;            right-arm (make-joint-goal '(0.52 -0.78 0.78 0.78 0.0 -0.78 0.0) nil))
;;           (roslisp-beasty:beasty-switch-behavior
;;            right-arm (make-joint-goal '(0.52 -0.78 0.78 0.78 0.0 -0.78 0.0) nil))
;;           (roslisp-beasty:move-beasty-and-wait 
;;            right-arm (make-joint-goal '(0.52 -0.78 0.78 0.78 0.0 -0.78 0.0) nil)))))
;;     (with-human-monitoring (human-percept) 
;;       (cpl-impl:on-suspension (roslisp-beasty:stop-beasty right-arm)
;;         (cpl-impl:retry-after-suspension
;;           (roslisp-beasty:beasty-safety-reset
;;            right-arm (make-joint-goal '(0.52 -0.78 0.78 0.78 0.0 -0.78 0.0) nil))
;;           (roslisp-beasty:beasty-switch-behavior
;;            right-arm (make-joint-goal '(0.52 -0.78 0.78 0.78 0.0 -0.78 0.0) nil))
;;           (roslisp-beasty:move-beasty-and-wait 
;;            right-arm 
;;            (make-cartesian-goal 
;;             (cl-transforms:make-transform
;;              (cl-transforms:make-3d-vector 0.359 0.459 0.533)
;;              (cl-transforms:axis-angle->quaternion
;;               (cl-transforms:make-3d-vector 0 1 0) PI)) nil)))))))

(defun main ()
  (with-ros-node ("saphari_demo")
    (test-suspend-motions (init-cram-saphari-demo))))

(defun dump-files ()
  (with-ros-node ("saphari_demo")
    (beliefstate:extract-files)))