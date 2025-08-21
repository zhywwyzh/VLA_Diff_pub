; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude kill_base.msg.html

(cl:defclass <kill_base> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass kill_base (<kill_base>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <kill_base>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'kill_base)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<kill_base> is deprecated: use traj_utils-msg:kill_base instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <kill_base>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <kill_base>) ostream)
  "Serializes a message object of type '<kill_base>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <kill_base>) istream)
  "Deserializes a message object of type '<kill_base>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<kill_base>)))
  "Returns string type for a message object of type '<kill_base>"
  "traj_utils/kill_base")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'kill_base)))
  "Returns string type for a message object of type 'kill_base"
  "traj_utils/kill_base")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<kill_base>)))
  "Returns md5sum for a message object of type '<kill_base>"
  "6cd4b8fbb9ccb430cf5df0ceccf8d731")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'kill_base)))
  "Returns md5sum for a message object of type 'kill_base"
  "6cd4b8fbb9ccb430cf5df0ceccf8d731")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<kill_base>)))
  "Returns full string definition for message of type '<kill_base>"
  (cl:format cl:nil "int8 drone_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'kill_base)))
  "Returns full string definition for message of type 'kill_base"
  (cl:format cl:nil "int8 drone_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <kill_base>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <kill_base>))
  "Converts a ROS message object to a list"
  (cl:list 'kill_base
    (cl:cons ':drone_id (drone_id msg))
))
