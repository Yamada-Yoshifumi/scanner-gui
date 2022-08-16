
(cl:in-package :asdf)

(defsystem "ros_srv-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ImuSwitch" :depends-on ("_package_ImuSwitch"))
    (:file "_package_ImuSwitch" :depends-on ("_package"))
    (:file "VelodyneSwitch" :depends-on ("_package_VelodyneSwitch"))
    (:file "_package_VelodyneSwitch" :depends-on ("_package"))
  ))