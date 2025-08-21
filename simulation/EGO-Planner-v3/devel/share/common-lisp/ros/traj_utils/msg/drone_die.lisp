; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude drone_die.msg.html

(cl:defclass <drone_die> (roslisp-msg-protocol:ros-message)
  ((drone_die
    :reader drone_die
    :initarg :drone_die
    :type cl:integer
    :initform 0))
)

(cl:defclass drone_die (<drone_die>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drone_die>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drone_die)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<drone_die> is deprecated: use traj_utils-msg:drone_die instead.")))

(cl:ensure-generic-function 'drone_die-val :lambda-list '(m))
(cl:defmethod drone_die-val ((m <drone_die>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_die-val is deprecated.  Use traj_utils-msg:drone_die instead.")
  (drone_die m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drone_die>) ostream)
  "Serializes a message object of type '<drone_die>"
  (cl:let* ((signed (cl:slot-value msg 'drone_die)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drone_die>) istream)
  "Deserializes a message object of type '<drone_die>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_die) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drone_die>)))
  "Returns string type for a message object of type '<drone_die>"
  "traj_utils/drone_die")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drone_die)))
  "Returns string type for a message object of type 'drone_die"
  "traj_utils/drone_die")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drone_die>)))
  "Returns md5sum for a message object of type '<drone_die>"
  "469265ba708544468cfac3f514285ad9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drone_die)))
  "Returns md5sum for a message object of type 'drone_die"
  "469265ba708544468cfac3f514285ad9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drone_die>)))
  "Returns full string definition for message of type '<drone_die>"
  (cl:format cl:nil "int32 drone_die~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drone_die)))
  "Returns full string definition for message of type 'drone_die"
  (cl:format cl:nil "int32 drone_die~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drone_die>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drone_die>))
  "Converts a ROS message object to a list"
  (cl:list 'drone_die
    (cl:cons ':drone_die (drone_die msg))
))
