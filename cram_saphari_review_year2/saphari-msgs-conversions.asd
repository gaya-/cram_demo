; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem saphari-msgs-conversions
  :name "saphari-msgs-conversions"
  :author "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :version "0.1"
  :maintainer "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :licence "BSD"
  :description "Conversion functions for ROS saphari_msgs."
  :depends-on (:roslisp :alexandria :saphari_msgs-msg :cl-tf2)
  :components
  ((:module "src"
    :components
    ((:module "saphari-msgs-conversions"
      :components
      ((:file "package")
       (:file "conversions" :depends-on ("package"))))))))