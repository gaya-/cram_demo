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

(cut:define-hook cram-language::on-start-human-tracking (human-desig))
(cut:define-hook cram-language::on-stop-human-tracking (log-id human-desig))

(defparameter *humans-fluent* 
  (cram-language-implementation:make-fluent :name "humans-fluent"))

(defparameter *tracker-fluent* nil)

(defparameter *test-percept0*
  `(:user-id -1 :stamp 1.0 :frame-id "some-frame" :bodyparts nil))

(defparameter *test-percept1*
  `(:user-id -1 :stamp 2.0 :frame-id "some-frame" :bodyparts nil))

(defparameter *test-percept2*
  `(:user-id 1 :stamp 3.0 :frame-id "some-frame" :bodyparts (:knee :hand)))

(defparameter *test-percept3*
  `(:user-id 1 :stamp 4.0 :frame-id "some-frame" :bodyparts (:knee :hand)))

(defparameter *test-percept4*
  `(:user-id -1 :stamp 5.0 :frame-id "some-frame" :bodyparts nil))

(defparameter *test-percepts*
  (list *test-percept0* *test-percept1* *test-percept2* *test-percept3* *test-percept4*))

(defparameter *test-desig0*
  (desig:make-designator 'desig:human '((:tracker :openni))))

(defparameter *test-desig1*
  (desig:make-effective-designator
   *test-desig0*
   :new-properties (desig:merge-desig-descriptions *test-desig0* '((:user-id 1)))
   :data-object *test-percept2*))

(defun human-detected-p (human-percept)
  "Predicate to check whether `human-percept' represents a detected human."
  (not (= (getf human-percept :user-id) -1)))

(defun human-percept-matches-desig-p (human-percept human-desig)
  "Predicate checking whether `human-percept' matches the description of `human-desig'."
  (and (= (desig:desig-prop-value human-desig :user-id)
          (getf human-percept :user-id))))

(defun find-matching-human-designator (human-percept human-desigs)
  "Returns the designator in `human-desigs' which matches `human-percept'."
  (find human-percept human-desigs :test #'human-percept-matches-desig-p))