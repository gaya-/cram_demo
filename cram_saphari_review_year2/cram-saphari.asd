; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-saphari
  :name "cram-saphari"
  :author "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :version "0.0"
  :maintainer "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :licence "BSD"
  :description "CRAM code for Saphari development"
  :depends-on (:cram-reasoning
               :cram-beasty)
  :components
  ((:module "cram-saphari"
            :components
            ((:file "package")
             (:file "beasty-predicates" :depends-on ("package"))))))