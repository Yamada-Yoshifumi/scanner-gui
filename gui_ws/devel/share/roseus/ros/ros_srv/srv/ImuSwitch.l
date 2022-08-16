;; Auto-generated. Do not edit!


(when (boundp 'ros_srv::ImuSwitch)
  (if (not (find-package "ROS_SRV"))
    (make-package "ROS_SRV"))
  (shadow 'ImuSwitch (find-package "ROS_SRV")))
(unless (find-package "ROS_SRV::IMUSWITCH")
  (make-package "ROS_SRV::IMUSWITCH"))
(unless (find-package "ROS_SRV::IMUSWITCHREQUEST")
  (make-package "ROS_SRV::IMUSWITCHREQUEST"))
(unless (find-package "ROS_SRV::IMUSWITCHRESPONSE")
  (make-package "ROS_SRV::IMUSWITCHRESPONSE"))

(in-package "ROS")





(defclass ros_srv::ImuSwitchRequest
  :super ros::object
  :slots (_command ))

(defmethod ros_srv::ImuSwitchRequest
  (:init
   (&key
    ((:command __command) 0)
    )
   (send-super :init)
   (setq _command (round __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; int64 _command
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _command
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _command (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _command) (= (length (_command . bv)) 2)) ;; bignum
              (write-long (ash (elt (_command . bv) 0) 0) s)
              (write-long (ash (elt (_command . bv) 1) -1) s))
             ((and (class _command) (= (length (_command . bv)) 1)) ;; big1
              (write-long (elt (_command . bv) 0) s)
              (write-long (if (>= _command 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _command s)(write-long (if (>= _command 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _command
#+(or :alpha :irix6 :x86_64)
      (setf _command (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _command (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass ros_srv::ImuSwitchResponse
  :super ros::object
  :slots (_success ))

(defmethod ros_srv::ImuSwitchResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ros_srv::ImuSwitch
  :super ros::object
  :slots ())

(setf (get ros_srv::ImuSwitch :md5sum-) "0f0513142f201da5c030b442ad26bf2d")
(setf (get ros_srv::ImuSwitch :datatype-) "ros_srv/ImuSwitch")
(setf (get ros_srv::ImuSwitch :request) ros_srv::ImuSwitchRequest)
(setf (get ros_srv::ImuSwitch :response) ros_srv::ImuSwitchResponse)

(defmethod ros_srv::ImuSwitchRequest
  (:response () (instance ros_srv::ImuSwitchResponse :init)))

(setf (get ros_srv::ImuSwitchRequest :md5sum-) "0f0513142f201da5c030b442ad26bf2d")
(setf (get ros_srv::ImuSwitchRequest :datatype-) "ros_srv/ImuSwitchRequest")
(setf (get ros_srv::ImuSwitchRequest :definition-)
      "int64 command
---
bool success

")

(setf (get ros_srv::ImuSwitchResponse :md5sum-) "0f0513142f201da5c030b442ad26bf2d")
(setf (get ros_srv::ImuSwitchResponse :datatype-) "ros_srv/ImuSwitchResponse")
(setf (get ros_srv::ImuSwitchResponse :definition-)
      "int64 command
---
bool success

")



(provide :ros_srv/ImuSwitch "0f0513142f201da5c030b442ad26bf2d")


