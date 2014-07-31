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
;;; ARM REASONING --> GO TO DESIGNATORS ?
;;;

(defun arm->frame (arm)
  (let ((bindings (cram-utilities:lazy-car
                   (crs:prolog `(arm-gripper-frame ,arm ?frame)))))
    (when bindings (cram-utilities:var-value '?frame bindings))))

(defun arm->gripper (arm)
  (let ((bindings (cram-utilities:lazy-car
                   (crs:prolog `(gripper-arm ?gripper ,arm)))))
    (when bindings (cram-utilities:var-value '?gripper bindings))))

;;;
;;; CONTROLLER MACRO --> GO TO CONTROLLERS ?
;;;

(defmacro with-vel-controllers (&body body)
  `(progn (ensure-vel-controllers)
          (unwind-protect ,@body
            (ensure-pos-controllers))))

;;;
;;; TF MACRO --> GO TO T ?
;;;

(defmacro with-desig-tf-broadcasting ((&rest desigs) &body body)
  `(cl-tf::with-tf-broadcasting-list
      ((get-tf-broadcaster)
       (mapcar
        (cl-utils:compose
         #'construct-desig-transform
         #'prepare-desig-pose-for-manipulation)
        (list ,@desigs)))
     (cpl-impl:sleep* 2)
     ,@body))

(defun find-tf-thread ()
  (find-if (lambda (name)
             (search "static broadcaster" name))
           (sb-thread:list-all-threads)
           :key #'sb-thread:thread-name))
;;;
;;; AUX PLANS --> ???
;;;

(cpl-impl:def-cram-function execute-single-arm-constraint-motion 
    (motion fluent start stop)
  (cram-language:pursue
    (funcall start motion)
    (cram-language:whenever ((cram-language:pulsed fluent))
      (when (cram-language:value fluent)
        (funcall stop)))))

(cpl-impl:def-cram-function execute-dual-arm-constraint-motion 
    (l-motion l-fluent l-start l-stop r-motion r-fluent r-start r-stop)
  (let ((combined-fluent (cpl-impl:fl-and l-fluent r-fluent)))
    (cram-language:par
      (execute-single-arm-constraint-motion l-motion combined-fluent l-start l-stop)
      (execute-single-arm-constraint-motion r-motion combined-fluent r-start r-stop))))

;;;
;;; POURING
;;;

(cpl-impl:def-cram-function sim-pouring-test (pouring-arm)
  (move-into-pouring-configuration pouring-arm)
  (let ((pancake-bottle-pose
          (cl-tf:make-pose-stamped
           (arm->frame pouring-arm) 0
           (cl-transforms:make-3d-vector -0.01 0 0)
           (cl-transforms:make-identity-rotation)))
        (pancake-maker-pose-msg
          (cl-tf:pose-stamped->msg
           (cl-tf:make-pose-stamped
            "table" 0
            (cl-transforms:make-3d-vector -0.1 0 0.75)
            (cl-transforms:make-identity-rotation)))))
    (with-designators
        ((pancake-maker (cram-designators:object
                         `((type "Pancake_maker")
                           (desig-props:pose ,pancake-maker-pose-msg))))
         (pancake-bottle (cram-designators:object
                          `((type "pancake_mix")
                            (desig-props:pose ,pancake-bottle-pose)
                            (desig-props:in ,(arm->gripper pouring-arm))
                            (desig-props:child-frame-id "pancake_bottle")))))
      (demo-part-pouring pancake-bottle pancake-maker))))

(cpl-impl:def-cram-function real-pouring-test (pouring-arm)
  (move-into-pouring-configuration pouring-arm)
  (let* ((pancake-bottle-pose
           (cl-tf:make-pose-stamped
            (arm->frame pouring-arm) 0
            (cl-transforms:make-3d-vector -0.01 0 0)
            (cl-transforms:make-identity-rotation)))
         (percepts (cram-uima:query-uima-and-wait (get-uima)))
         (pancake-maker (find-desig percepts 'desig-props:type "Pancake_maker")))
    (unless pancake-maker (cpl-impl:fail "Did not detect pancake maker."))
    (with-designators
        ((pancake-bottle (cram-designators:object               
                          `((type "pancake_mix")
                            (desig-props:pose ,pancake-bottle-pose)
                            (desig-props:in ,(arm->gripper pouring-arm))
                            (desig-props:child-frame-id "pancake_bottle")))))
      (demo-part-pouring pancake-bottle pancake-maker))))

(cpl-impl:def-cram-function demo-part-pouring (pancake-mix pancake-maker)
  (with-designators ((desig (action `((type constraints) 
                                      (to pour)
                                      (obj-acted-with ,pancake-mix)))))
    (destructuring-bind (motions start-controller stop-controller finished-fluent)
        (reference desig)
      (with-desig-tf-broadcasting (pancake-mix pancake-maker)
        (with-vel-controllers
          (loop for motion in motions do
            (add-motion-phase-to-designator-description desig (id motion))
            (execute-single-arm-constraint-motion 
             motion finished-fluent start-controller stop-controller)))))))

;;;
;;; FLIPPING
;;;

(cpl-impl:def-cram-function spatula-calibration ()
  (let ((l-spatula-pose
           (cl-tf:make-pose-stamped
            "l_gripper_tool_frame" 0
            (cl-transforms:make-3d-vector -0.01 0.0 0.0)
            (cl-transforms:make-identity-rotation)))
         (r-spatula-pose
           (cl-tf:make-pose-stamped
            "r_gripper_tool_frame" 0
            (cl-transforms:make-3d-vector 0.0 0.0 0.0)
            (cl-transforms:q*
             (cl-transforms:axis-angle->quaternion
              (cl-transforms:make-3d-vector 0 1 0) 0.1)
             (cl-transforms:axis-angle->quaternion 
              (cl-transforms:make-3d-vector 1 0 0) PI)))))
    (with-designators
        ((l-spatula (cram-designators:object
                     `((type "spatula")
                       (desig-props:in "l_gripper")
                       (desig-props:child-frame-id "l_spatula_handle")
                       (desig-props:pose ,l-spatula-pose))))
         (r-spatula (cram-designators:object
                     `((type "spatula")
                       (desig-props:in "r_gripper")
                       (desig-props:child-frame-id "r_spatula_handle")
                       (desig-props:pose ,r-spatula-pose)))))
      (with-desig-tf-broadcasting (l-spatula r-spatula) 
        nil))))

(cpl-impl:def-cram-function sim-flipping-test ()
  (let* ((pancake-maker-pose-msg
          (cl-tf:pose-stamped->msg
           (cl-tf:make-pose-stamped
            "table" 0
            (cl-transforms:make-3d-vector -0.1 0.0 0.75)
            (cl-transforms:make-identity-rotation))))
        (pancake-pose-msg pancake-maker-pose-msg) ; for simplicity's sake
        (l-spatula-pose
          (cl-tf:make-pose-stamped
           "l_gripper_tool_frame" 0
            (cl-transforms:make-3d-vector -0.03 0.0 0.0)
            (cl-transforms:make-identity-rotation)))
        (r-spatula-pose
          (cl-tf:make-pose-stamped
           "r_gripper_tool_frame" 0
            (cl-transforms:make-3d-vector -0.03 0.0 0.0)
            (cl-transforms:make-identity-rotation))))
    (with-designators
        ((pancake-maker (cram-designators:object
                         `((type "Pancake_maker")
                           (desig-props:pose ,pancake-maker-pose-msg))))
         (pancake (cram-designators:object
                   `((type "pancake")
                     (desig-props:pose ,pancake-pose-msg))))
         (l-spatula (cram-designators:object
                     `((type "spatula")
                       (desig-props:in "l_gripper")
                       (desig-props:child-frame-id "l_spatula_handle")
                       (desig-props:pose ,l-spatula-pose))))
         (r-spatula (cram-designators:object
                     `((type "spatula")
                       (desig-props:in "r_gripper")
                       (desig-props:child-frame-id "r_spatula_handle")
                       (desig-props:pose ,r-spatula-pose)))))
      (demo-part-flipping l-spatula r-spatula pancake pancake-maker))))

(cpl-impl:def-cram-function real-flipping-test ()
  (move-into-flipping-configuration)
  (let* ((l-spatula-pose
           (cl-tf:make-pose-stamped
            "l_gripper_tool_frame" 0
            (cl-transforms:make-3d-vector -0.01 0.0 0.0)
            (cl-transforms:make-identity-rotation)))
         (r-spatula-pose
           (cl-tf:make-pose-stamped
            "r_gripper_tool_frame" 0
            (cl-transforms:make-3d-vector 0.0 0.0 0.0)
            (cl-transforms:q*
             (cl-transforms:axis-angle->quaternion
              (cl-transforms:make-3d-vector 0 1 0) 0.1)
             (cl-transforms:axis-angle->quaternion 
              (cl-transforms:make-3d-vector 1 0 0) PI))))
         (percepts (cram-uima:query-uima-and-wait (get-uima)))
         (pancake (find-desig percepts 'desig-props:type "pancake"))
         (pancake-maker (find-desig percepts 'desig-props:type "Pancake_maker")))
    (with-designators
        ((l-spatula (cram-designators:object
                     `((type "spatula")
                       (desig-props:in "l_gripper")
                       (desig-props:child-frame-id "l_spatula_handle")
                       (desig-props:pose ,l-spatula-pose))))
         (r-spatula (cram-designators:object
                     `((type "spatula")
                       (desig-props:in "r_gripper")
                       (desig-props:child-frame-id "r_spatula_handle")
                       (desig-props:pose ,r-spatula-pose)))))
      (unless pancake (cpl-impl:fail "Did not detect pancake."))
      (unless pancake-maker (cpl-impl:fail "Did not detect pancake maker."))
      (demo-part-flipping l-spatula r-spatula pancake pancake-maker))))

(cpl-impl:def-cram-function demo-part-flipping 
    (l-spatula r-spatula pancake pancake-maker)
  (with-designators ((desig (action `((type constraints) (to flip)))))
    (destructuring-bind (l-motions l-start-controller l-stop-controller l-finished-fluent
                         r-motions r-start-controller r-stop-controller r-finished-fluent)
        (reference desig)
      (with-desig-tf-broadcasting (pancake pancake-maker l-spatula r-spatula)
        (with-vel-controllers
          (loop for l-motion in l-motions
                for r-motion in r-motions do
                  (execute-dual-arm-constraint-motion
                   l-motion l-finished-fluent l-start-controller l-stop-controller
                   r-motion r-finished-fluent r-start-controller r-stop-controller)))))))

;;;
;;; GRASPING
;;;

(cpl-impl:def-cram-function demo-part-grasping ()
  (with-designators ((desig (action `((type constraints) (to grasp)))))
    (destructuring-bind (motions l-start-controller l-stop-controller l-finished-fluent)
        (reference desig)
      (ensure-vel-controllers)
      (loop for motion in motions do
        (format t "PERFORMING SOME MOTION.~%")
        (cram-language:pursue
          (funcall l-start-controller motion)
          (cram-language:whenever ((cram-language:pulsed l-finished-fluent))
            (when (cram-language-implementation:value l-finished-fluent)
              (funcall l-stop-controller)))))
      (ensure-pos-controllers))))

(cpl-impl:def-cram-function move-into-grasping-configuration ()
  (ensure-pos-controllers)
  (cram-language:par
    (pr2-controllers:move-arm 
     (get-right-arm-position-controller)
     *r-arm-grasping-configuration* 4.0)
    (pr2-controllers:move-arm
     (get-left-arm-position-controller)
     *l-arm-grasping-configuration* 4.0)))

;;;
;;; PLAY THINGS
;;;

(defun move-arm-into-config (arm config-lookup-fun time)
  (pr2-controllers:move-arm 
   (get-position-controller arm)
   (funcall config-lookup-fun arm) time))

(cpl-impl:def-cram-function move-arms-with-config-fun (config-lookup-fun time)
  (ensure-pos-controllers)
  (cram-language:par
    (move-arm-into-config 'left-arm config-lookup-fun time)
    (move-arm-into-config 'right-arm config-lookup-fun time)))

(cpl-impl:def-cram-function move-into-resting-configuration (&optional (time 4.0))
   (move-arms-with-config-fun #'resting-config time))

(cpl-impl:def-cram-function move-into-flipping-configuration (&optional (time 4.0))
  (move-arms-with-config-fun #'flipping-config time))

(cpl-impl:def-cram-function move-into-pouring-configuration (pouring-arm &optional
                                                                         (time 4.0))
  (move-arms-with-config-fun 
   (lambda (arm) (get-pouring-start-config pouring-arm arm)) time))

;;;
;;; DESIGNATOR LOGGING
;;;

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

;;;
;;; SOME TF MAGIC 
;;;

(defun transform-pose-ignoring-stamp (pose-stamped target-frame)
  (cl-tf:transform-pose 
   (get-tf-listener)
   :pose (cl-tf:copy-pose-stamped pose-stamped :stamp 0.0)
   :target-frame target-frame))

(defun pose-stamped->transform-stamped (pose-stamped child-frame-id)
  (with-slots (cl-tf:frame-id cl-tf:stamp cl-tf:origin cl-tf:orientation)
      pose-stamped
    (cl-tf:make-stamped-transform
     cl-tf:frame-id child-frame-id cl-tf:stamp cl-tf:origin cl-tf:orientation)))

;;;
;;; SOME ORIENTATION MAGIC
;;;

(defun change-pose-orientation (pose orientation)
  (cl-tf:copy-pose-stamped pose :orientation orientation))

(defun calculate-pointing-orientation (pose)
  (let* ((x-rot (cl-transforms:copy-3d-vector
                 (cl-transforms:origin pose) :z 0.0))
         (z-rot (cl-transforms:make-3d-vector 0 0 1))
         (y-rot (cl-transforms:cross-product z-rot x-rot)))
    (cl-transforms:column-vectors->quaternion x-rot y-rot z-rot)))

(defun change-pose-orientation-to-pointing (pose)
  (change-pose-orientation pose (calculate-pointing-orientation pose)))
  
;;;
;;; POSES IN AND OUT OF DESIGS
;;;

(defun ensure-pose-stamped (pose)
  (etypecase pose
    (cl-tf:pose-stamped pose)
    (geometry_msgs-msg:posestamped (cl-tf:msg->pose-stamped pose))))

(defun get-desig-pose (desig)
  (ensure-pose-stamped
   (or
    (cram-designators:desig-prop-value desig 'desig-props:pose)
    (cram-designators:desig-prop-value desig 'desig-props::pose-center)
    (cram-designators:desig-prop-value desig 'desig-props::pose-on-plane))))

(defun set-desig-pose-in-data (obj-desig pose)
  (with-slots (cram-designators:data) obj-desig
    (setf cram-designators:data pose))
  obj-desig)

;;;
;;; HIGH-LEVEL DESIG MAGIC
;;;

(defun in-gripper-p (desig)
  (gripper-p (cram-designators:desig-prop-value desig 'desig-props:in)))

(defun gripper-p (string-id)
  (or (string= "l_gripper" string-id) (string= "r_gripper" string-id)))

(defun prepare-desig-pose-for-manipulation (desig)
  (set-desig-pose-in-data
   desig
   ;;; 
   (if (in-gripper-p desig)
       ;; the poses of objects in hand can just be taken from the description
       (get-desig-pose desig)
       ;; all the perceived objects: we need to transform and slightly change their poses
       (change-pose-orientation-to-pointing
        (transform-pose-ignoring-stamp 
         (get-desig-pose desig)
         "base_link")))))

(defun construct-desig-transform (desig)
  (pose-stamped->transform-stamped 
   (reference desig) 
   (string-downcase 
    (or
     (cram-designators:desig-prop-value desig 'desig-props:child-frame-id)
     (cram-designators:desig-prop-value desig 'desig-props:type)))))

;;;
;;; FINDING THE RIGHT DESIGNATOR
;;;

(defun find-desig (desigs property desired-value)
  (find desired-value desigs 
        :test #'equal 
        :key (lambda (desig) (cram-designators:desig-prop-value desig property))))

(defun find-desigs (desigs property desired-value)
  (remove-if-not 
   (lambda (value) (equal value desired-value))
   desigs
   :key (lambda (desig) (cram-designators:desig-prop-value desig property))))