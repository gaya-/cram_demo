; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem human-state-publisher
  :name "human-state-publisher"
  :author "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :version "0.1"
  :maintainer "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :licence "BSD"
  :description "A tool to publish the detected link poses of a human to tf."
  :depends-on (:roslisp :alexandria :cl-tf2 :saphari_msgs-msg :saphari-msgs-conversions)
  :components
  ((:module "src"
    :components
    ((:module "human-state-publisher"
      :components
      ((:file "package")
       (:file "message-conversions" :depends-on ("package"))
       (:file "human-state-publisher" :depends-on ("package" "message-conversions"))))))))
