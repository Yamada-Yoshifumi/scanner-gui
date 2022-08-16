; Auto-generated. Do not edit!


(cl:in-package ros_srv-srv)


;//! \htmlinclude ImuSwitch-request.msg.html

(cl:defclass <ImuSwitch-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:integer
    :initform 0))
)

(cl:defclass ImuSwitch-request (<ImuSwitch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuSwitch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuSwitch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_srv-srv:<ImuSwitch-request> is deprecated: use ros_srv-srv:ImuSwitch-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <ImuSwitch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_srv-srv:command-val is deprecated.  Use ros_srv-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuSwitch-request>) ostream)
  "Serializes a message object of type '<ImuSwitch-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuSwitch-request>) istream)
  "Deserializes a message object of type '<ImuSwitch-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuSwitch-request>)))
  "Returns string type for a service object of type '<ImuSwitch-request>"
  "ros_srv/ImuSwitchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuSwitch-request)))
  "Returns string type for a service object of type 'ImuSwitch-request"
  "ros_srv/ImuSwitchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuSwitch-request>)))
  "Returns md5sum for a message object of type '<ImuSwitch-request>"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuSwitch-request)))
  "Returns md5sum for a message object of type 'ImuSwitch-request"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuSwitch-request>)))
  "Returns full string definition for message of type '<ImuSwitch-request>"
  (cl:format cl:nil "int64 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuSwitch-request)))
  "Returns full string definition for message of type 'ImuSwitch-request"
  (cl:format cl:nil "int64 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuSwitch-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuSwitch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuSwitch-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude ImuSwitch-response.msg.html

(cl:defclass <ImuSwitch-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ImuSwitch-response (<ImuSwitch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuSwitch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuSwitch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_srv-srv:<ImuSwitch-response> is deprecated: use ros_srv-srv:ImuSwitch-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ImuSwitch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_srv-srv:success-val is deprecated.  Use ros_srv-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuSwitch-response>) ostream)
  "Serializes a message object of type '<ImuSwitch-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuSwitch-response>) istream)
  "Deserializes a message object of type '<ImuSwitch-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuSwitch-response>)))
  "Returns string type for a service object of type '<ImuSwitch-response>"
  "ros_srv/ImuSwitchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuSwitch-response)))
  "Returns string type for a service object of type 'ImuSwitch-response"
  "ros_srv/ImuSwitchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuSwitch-response>)))
  "Returns md5sum for a message object of type '<ImuSwitch-response>"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuSwitch-response)))
  "Returns md5sum for a message object of type 'ImuSwitch-response"
  "0f0513142f201da5c030b442ad26bf2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuSwitch-response>)))
  "Returns full string definition for message of type '<ImuSwitch-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuSwitch-response)))
  "Returns full string definition for message of type 'ImuSwitch-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuSwitch-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuSwitch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuSwitch-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ImuSwitch)))
  'ImuSwitch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ImuSwitch)))
  'ImuSwitch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuSwitch)))
  "Returns string type for a service object of type '<ImuSwitch>"
  "ros_srv/ImuSwitch")