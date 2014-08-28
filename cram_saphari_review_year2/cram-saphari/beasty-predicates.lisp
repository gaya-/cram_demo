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

(in-package :cram-saphari)

;;;
;;; SOME UTILS I DID NOT FIND IN CRAM-REASONING
;;;

(defun substitution-predicate-functor (elem key)
  (lambda (e) (equal e (funcall key elem))))

(def-fact-group reasoning-utilities (to-set has-duplicates copy-expand substitute-if)
  
  (<- (to-set ?list ?set)
    (lisp-fun remove-duplicates ?list :test equal ?set))

  (<- (has-duplicates ?list)
    (to-set ?list ?set)
    (lisp-fun length ?list ?list-length)
    (lisp-fun length ?set ?set-length)
    (not (== ?list-length ?set-length)))
  
  (<- (copy-expand ?list ?expanded-list)
    (bound ?list)
    (lisp-fun copy-list ?list ?copy)
    (lisp-fun cut:force-ll ?copy ?expanded-list))

  (<- (substitute-if ?elem ?list ?key ?result)
    (lisp-fun substitution-predicate-functor ?elem ?key ?pred)
    (lisp-fun substitute-if ?elem ?pred ?list :key ?key ?result)))

;;;
;;; REASONING ABOUT BEASTY SAFETY SETTINGS
;;; 

(defparameter *beasty-collision-types*
  '(:NO-CONTACT :CONTACT :LIGHT-COLLISION :STRONG-COLLISION :SEVERE-COLLISION)
  "NOTE: Order is important and will change behavior of collision sorting.")

(defparameter *beasty-reaction-types*
  '(:IGNORE :ZERO-G :JOINT-IMP :SOFT-STOP :HARD-STOP)
  "NOTE: Order is important and will change behavior of reaction sorting.")

(defparameter *default-beasty-safety-settings*
  '((:NO-CONTACT :IGNORE)
    (:CONTACT :IGNORE)
    (:LIGHT-COLLISION :JOINT-IMP)
    (:STRONG-COLLISION :SOFT-STOP)
    (:SEVERE-COLLISION :HARD-STOP)))

(defun beasty-collision-types () *beasty-collision-types*)

(defun beasty-reaction-types () *beasty-reaction-types*)

(defun default-beasty-safety-settings () *default-beasty-safety-settings*)

(defun weaker-collision-p (coll-a coll-b)
  "Predicate to sort list of collisions. Returns true if `coll-a' is a strictly
 weaker collision than `coll-b'."
  (reduce #'< 
          (mapcar (lambda (c) (position c (beasty-collision-types)))     
                  (list coll-a coll-b))))

(defun weaker-reaction-p (reac-a reac-b)
  "Predicate to sort list of reactions. Returns true if `reac-a' is a strictly
 weaker reaction than `reac-b'."
  (reduce #'<
          (mapcar (lambda (r) (position r (beasty-reaction-types)))
                  (list reac-a reac-b))))

(def-fact-group safety-settings 
    (beasty-collision beasty-reaction beasty-reflex 
                      beasty-safety-settings valid-beasty-safety-settings
                      cautious-beasty-safety-settings)

   (<- (beasty-collision ?collision)
     (lisp-fun beasty-collision-types ?collision-types)     
     (member ?collision ?collision-types))

  (<- (beasty-collision (?collision ?_) ?collision)
    (beasty-collision ?collision))

  (<- (beasty-reaction ?reaction)
    (lisp-fun beasty-reaction-types ?reaction-types)
    (member ?reaction ?reaction-types))

  (<- (beasty-reaction (?_ ?reaction) ?reaction)
    (beasty-reaction ?reaction))

  (<- (beasty-reflex (?collision ?reaction))
    (beasty-collision ?collision)
    (beasty-reaction ?reaction))

  (<- (beasty-safety-settings ?settings)
    (lisp-type ?settings list)
    (forall (member ?reflex ?settings) (beasty-reflex ?reflex)))

  (<- (valid-beasty-safety-settings ?settings)
    ;; is a list of unique beasty reflexes
    (beasty-safety-settings ?settings)
    (not (has-duplicates ?settings))
    ;; has no duplicated collisions to trigger reactions
    (lisp-fun mapcar first ?settings ?collisions)
    (not (has-duplicates ?collisions))
    ;; reactions escalate monotonously
    (sort ?settings weaker-collision-p first ?sorted-settings)
    (lisp-fun mapcar second ?sorted-settings ?reactions)
    (sort ?reactions weaker-reaction-p ?sorted-reactions)
    (equal ?reactions ?sorted-reactions))

  (<- (default-beasty-safety-settings ?settings)
    (lisp-fun default-beasty-safety-settings ?default-settings)
    (equal ?settings ?default-settings))

  (<- (cautious-beasty-safety-settings ?settings)
    (default-beasty-safety-settings ?default-settings)
    (substitute-if (:CONTACT :JOINT-IMP) ?default-settings car ?cautious-settings)
    (equal ?cautious-settings ?settings)))