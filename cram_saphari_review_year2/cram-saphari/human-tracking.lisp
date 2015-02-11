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
;;; TEST DATA
;;;

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

(defparameter *test-desigs*
  (list *test-desig0* *test-desig1*))

;;;
;;; ACTUAL HUMAN DESIGNATOR STUFF
;;;

(defun human-detected-p (human-percept)
  "Predicate to check whether `human-percept' represents a detected human."
  (and (getf human-percept :user-id)
       (not (= (getf human-percept :user-id) -1))))

(defun human-percept-matches-desig-p (human-percept human-desig)
  "Predicate checking whether `human-percept' matches the description of `human-desig'."
  (and (getf human-percept :user-id)
       (desig:desig-prop-value human-desig :user-id)
       (= (desig:desig-prop-value human-desig :user-id)
          (getf human-percept :user-id))))

(defun human-desig-too-old-p (desig now timeout)
  (declare (type cram-designators:human-designator desig))
  (let ((last-timestamp (desig:desig-prop-value desig :last-detected)))
    (and last-timestamp (> (- now last-timestamp) timeout))))

(defun find-matching-human-desig (human-percept human-desigs)
  "Returns the designator in `human-desigs' which matches `human-percept'."
  (find human-percept human-desigs :test #'human-percept-matches-desig-p))

(defun remove-old-human-desigs (desigs timeout now)
  "Checks whether any of the human designators in `desigs' has not been detected 
 for more than `timeout' seconds. Uses `now' as the current timestamp. Any designator 
 that is old will be removed from `desigs'. Returns the new list of desigs."
  (declare (type list desigs))
  (remove-if (alexandria:rcurry #'human-desig-too-old-p now timeout) desigs))

(defun add-new-human-desig (percept desigs)
  "Creates a new human designator out of `percept' and appends it to the list
 human designators `desigs'. `percept' shall be a plist. Returns the new `desigs'."
  (declare (type list percept desigs))
  (cpl-desig-supp:with-designators 
      ((new-desig (desig:human `((:tracker :openni)
                                 (:user-id ,(getf percept :user-id))
                                 (:first-detected ,(getf percept :stamp))
                                 (:last-detected ,(getf percept :stamp))))))
    (let ((effective-desig
            (desig:equate 
             new-desig (desig:make-effective-designator new-desig :data-object percept))))
      (cons effective-desig desigs))))

(defun update-existing-desig (percept desigs)
  "Updates the first human designator in the list of human designators `desigs' which
 matches `percept' with a new effective designator. Assumes such a matching designator
 exists. Returns the new `desigs'."
  (let* ((old-desig (find-matching-human-desig percept desigs))
         (new-description (cram-designators:merge-desig-descriptions old-desig `((:last-detected ,(getf percept :stamp)))))
         (new-desig (desig:make-effective-designator 
                     old-desig :new-properties new-description :data-object percept)))
    (desig:equate old-desig new-desig)
    (substitute new-desig old-desig desigs)))

(defun update-human-desigs (percept desigs &key (timeout 2.0) (now (roslisp:ros-time)))
  ;; TODO(Georg): comment me!
  (if (human-detected-p percept)
      (if (find-matching-human-desig percept desigs)
          ;; CASE 1: PERCEPT AND MATCHING DESIG -> UPDATE DESIG
          (update-existing-desig percept desigs)
          ;; CASE 2: PERCEPT AND NO MATCHING DESIG -> ADD DESIG
          (add-new-human-desig percept desigs))
      (if (not desigs)
          ;; CASE 3: NO PERCEPT AND NO DESIGS -> DO NOTHING 
          nil
          ;; CASE 4: NO PERCEPT AND SOME DESIGS -> MAYBE REMOVE DESIG
          (remove-old-human-desigs desigs timeout now))))

(declare-goal track (humans-fluent percepts-fluent)
  (declare (ignore humans-fluent percepts-fluent)))

(def-goal (track ?humans-fluent ?percepts-fluent)
  (declare (ignore ?humans-fluent ?percepts-fluent))
  (format t "Trying to track human from percepts~%"))

(defun main ()
  (with-ros-node ("saphari-demo" :spin t)
    (top-level
      (let ((humans-fluent 1)
            (percept-fluent (make-fluent :name "human-percept-fluent")))
        (subscribe "/saphari/human" "saphari_msgs/Human" #'from-msg)
        (track humans-fluent percept-fluent)))))