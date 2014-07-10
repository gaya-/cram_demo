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

(in-package :serapeum)

(defgeneric add-associations (description &rest associations))

(defgeneric remove-associations (description &rest keys))

(defgeneric find-association (description key))

(defgeneric get-association (description key))
(defgeneric contains-association-p (description key))

(defgeneric get-keys (description))

(defgeneric get-values (description))

(defgeneric copy-description (description))

(defun cherry-pick-associations (description &rest keys)
  (apply #'remove-associations (copy-description description)
         (remove-if (lambda (key) (member key keys)) (get-keys description))))

(defun find-association-with-error (description key)
  (multiple-value-bind (value key-present-p) 
      (find-association description key)
    (if key-present-p
        (values value key-present-p)
        (error "No key ~a in description ~a.~%" key description))))
 
(defun get-association-with-error (description key)
  (multiple-value-bind (value key-present-p) 
      (find-association description key)
    (if key-present-p
        value
        (error "No key ~a in description ~a.~%" key description))))

(defun get-in-association (description keys)
  "Returns the association inside a nested `description', by using the sequence
 of `keys' to make the actual lookup."
  (reduce #'get-association keys :initial-value description))