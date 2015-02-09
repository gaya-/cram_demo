; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem saphari-utility-nodes
  :name "saphari-utility-nodes"
  :author "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :version "0.1"
  :maintainer "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :licence "BSD"
  :description "Various utils to needed to run the SAPHARI demo."
  :depends-on (:roslisp :alexandria :saphari_msgs-msg :cl-tf2)
  :components
  ((:module "src"
    :components
    ((:module "saphari-utility-nodes"
      :components
      ((:file "package")
       (:file "message-conversions" :depends-on ("package"))
       (:file "human-tf-converter" :depends-on ("package" "message-conversions"))))))))