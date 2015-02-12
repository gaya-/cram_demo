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

(in-package :saphari-msgs-conversions)

;;;
;;; Conversions from saphari_msgs ROS messages.
;;;

(defgeneric from-msg (msg))

(defmethod from-msg ((msg saphari_msgs-msg:human))
  (with-fields (userid (stamp (stamp header)) (frame-id (frame_id header)) bodyparts) msg
    (unless (= userid -1)
      `(:user-id ,userid :stamp ,stamp :frame-id ,frame-id
        :bodyparts ,(mapcar #'from-msg (coerce bodyparts 'list))))))

(defmethod from-msg ((msg saphari_msgs-msg:bodypart))
  (with-fields (label centroid) msg
    `(:label ,(bodypart-code->keyword label) 
      :centroid ,(from-msg centroid))))

(defmethod from-msg ((msg geometry_msgs-msg:point32))
  (with-fields (x y z) msg
    (cl-transforms:make-3d-vector x y z)))

(defun bodypart-code->keyword (bodypart-code)
  (first
   (find bodypart-code
         (roslisp-msg-protocol:symbol-codes 'saphari_msgs-msg:bodypart) 
         :key #'rest :test #'=)))

;;;
;;; Conversions to TF.
;;;

(defun human->stamped-transforms (human)
  (when human
    (destructuring-bind (&key user-id stamp frame-id bodyparts) human
      (mapcar (rcurry #'bodypart->stamped-transform user-id stamp frame-id) bodyparts))))
           
(defun bodypart->stamped-transform (bodypart user-id stamp frame-id)
  (destructuring-bind (&key label centroid) bodypart
    (cl-tf2:make-stamped-transform 
     frame-id (make-bodypart-frame-id label user-id) stamp
     (cl-transforms:make-transform centroid (cl-transforms:make-identity-rotation)))))

(defun make-bodypart-frame-id (label user-id)
  (concatenate 'string (make-human-tf-prefix user-id) (string-downcase (string label))))

(defun make-human-tf-prefix (user-id)
  (concatenate 'string "/human" (write-to-string user-id) "/"))