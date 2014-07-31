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
;;; FCCL CONTROLLER FOR LEFT ARM
;;;

(defparameter *l-arm-fccl-controller* nil)
(defparameter *r-arm-fccl-controller* nil)

(defun ensure-left-arm-fccl-controller ()
  (unless *l-arm-fccl-controller*
    (setf *l-arm-fccl-controller*
          (cram-fccl:make-fccl-action-client 
           *l-arm-fccl-controller-action-name* *l-arm-kinematic-chain*))))

(defun ensure-right-arm-fccl-controller ()
  (unless *r-arm-fccl-controller*
    (setf *r-arm-fccl-controller*
          (cram-fccl:make-fccl-action-client 
           *r-arm-fccl-controller-action-name* *r-arm-kinematic-chain*))))

(defun get-left-arm-fccl-controller ()
  (ensure-left-arm-fccl-controller)
  *l-arm-fccl-controller*)

(defun get-right-arm-fccl-controller ()
  (ensure-right-arm-fccl-controller)
  *r-arm-fccl-controller*)

(defun get-fccl-controller (side)
  (ecase side
    (right-arm (get-right-arm-fccl-controller))
    (left-arm (get-left-arm-fccl-controller))))

;;;
;;; HANDLE TO PR2 CONTROLLER MANAGER
;;;

(defparameter *pr2-controller-manager* nil)

(defun ensure-pr2-controller-manager ()
  (unless *pr2-controller-manager*
    (setf *pr2-controller-manager* (pr2-controllers:make-pr2-controller-manager-handle))))

(defun get-pr2-controller-manager ()
  (ensure-pr2-controller-manager)
  *pr2-controller-manager*)

(defun ensure-vel-controllers ()
  (pr2-controllers:ensure-vel-controllers (get-pr2-controller-manager)))

(defun ensure-pos-controllers ()
  (pr2-controllers:ensure-pos-controllers (get-pr2-controller-manager)))

;;;
;;; HANDLE TO PR2 STANDARD POSITION CONTROLLERS
;;;

(defparameter *l-arm-position-controller* nil)
(defparameter *r-arm-position-controller* nil)

(defun ensure-left-arm-position-controller ()
  (unless *l-arm-position-controller*
    (setf *l-arm-position-controller*
          (cram-pr2-controllers:make-pr2-arm-position-controller-handle
           *l-arm-position-controller-action-name* *l-arm-joint-names*))))

(defun ensure-right-arm-position-controller ()
  (unless *r-arm-position-controller*
    (setf *r-arm-position-controller*
          (cram-pr2-controllers:make-pr2-arm-position-controller-handle
           *r-arm-position-controller-action-name* *r-arm-joint-names*))))

(defun ensure-position-controller (arm)
  (ecase arm
    (left-arm (ensure-left-arm-position-controller))
    (right-arm (ensure-right-arm-position-controller))))

(defun get-position-controller (arm)
  (ensure-position-controller arm)
  (ecase arm
    (left-arm *l-arm-position-controller*)
    (right-arm *r-arm-position-controller*)))

(defun get-left-arm-position-controller ()
  (ensure-left-arm-position-controller)
  *l-arm-position-controller*)

(defun get-right-arm-position-controller ()
  (ensure-right-arm-position-controller)
  *r-arm-position-controller*)