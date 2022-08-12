; Auto-generated. Do not edit!


(cl:in-package ros_srv-srv)


;//! \htmlinclude VelodyneSwitch-request.msg.html

(cl:defclass <VelodyneSwitch-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:integer
    :initform 0))
)

(cl:defclass VelodyneSwitch-request (<VelodyneSwitch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelodyneSwitch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelodyneSwitch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_srv-srv:<VelodyneSwitch-request> is deprecated: use ros_srv-srv:VelodyneSwitch-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <VelodyneSwitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_srv-srv:command-val is deprecated.  Use ros_srv-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelodyneSwitch-request>) ostream)
  "Serializes a message object of type '<VelodyneSwitch-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelodyneSwitch-request>) istream)
  "Deserializes a message object of type '<VelodyneSwitch-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelodyneSwitch-request>)))
  "Returns string type for a service object of type '<VelodyneSwitch-request>"
  "ros_srv/VelodyneSwitchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelodyneSwitch-request)))
  "Returns string type for a service object of type 'VelodyneSwitch-request"
  "ros_srv/VelodyneSwitchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelodyneSwitch-request>)))
  "Returns md5sum for a message object of type '<VelodyneSwitch-request>"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelodyneSwitch-request)))
  "Returns md5sum for a message object of type 'VelodyneSwitch-request"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelodyneSwitch-request>)))
  "Returns full string definition for message of type '<VelodyneSwitch-request>"
  (cl:format cl:nil "int64 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelodyneSwitch-request)))
  "Returns full string definition for message of type 'VelodyneSwitch-request"
  (cl:format cl:nil "int64 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelodyneSwitch-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelodyneSwitch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VelodyneSwitch-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude VelodyneSwitch-response.msg.html

(cl:defclass <VelodyneSwitch-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VelodyneSwitch-response (<VelodyneSwitch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelodyneSwitch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelodyneSwitch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_srv-srv:<VelodyneSwitch-response> is deprecated: use ros_srv-srv:VelodyneSwitch-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <VelodyneSwitch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_srv-srv:success-val is deprecated.  Use ros_srv-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelodyneSwitch-response>) ostream)
  "Serializes a message object of type '<VelodyneSwitch-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelodyneSwitch-response>) istream)
  "Deserializes a message object of type '<VelodyneSwitch-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelodyneSwitch-response>)))
  "Returns string type for a service object of type '<VelodyneSwitch-response>"
  "ros_srv/VelodyneSwitchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelodyneSwitch-response)))
  "Returns string type for a service object of type 'VelodyneSwitch-response"
  "ros_srv/VelodyneSwitchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelodyneSwitch-response>)))
  "Returns md5sum for a message object of type '<VelodyneSwitch-response>"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelodyneSwitch-response)))
  "Returns md5sum for a message object of type 'VelodyneSwitch-response"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelodyneSwitch-response>)))
  "Returns full string definition for message of type '<VelodyneSwitch-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelodyneSwitch-response)))
  "Returns full string definition for message of type 'VelodyneSwitch-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelodyneSwitch-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelodyneSwitch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VelodyneSwitch-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VelodyneSwitch)))
  'VelodyneSwitch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VelodyneSwitch)))
  'VelodyneSwitch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelodyneSwitch)))
  "Returns string type for a service object of type '<VelodyneSwitch>"
  "ros_srv/VelodyneSwitch")