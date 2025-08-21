; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude take_off.msg.html

(cl:defclass <take_off> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass take_off (<take_off>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <take_off>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'take_off)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<take_off> is deprecated: use traj_utils-msg:take_off instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <take_off>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <take_off>) ostream)
  "Serializes a message object of type '<take_off>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <take_off>) istream)
  "Deserializes a message object of type '<take_off>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<take_off>)))
  "Returns string type for a message object of type '<take_off>"
  "traj_utils/take_off")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'take_off)))
  "Returns string type for a message object of type 'take_off"
  "traj_utils/take_off")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<take_off>)))
  "Returns md5sum for a message object of type '<take_off>"
  "6cd4b8fbb9ccb430cf5df0ceccf8d731")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'take_off)))
  "Returns md5sum for a message object of type 'take_off"
  "6cd4b8fbb9ccb430cf5df0ceccf8d731")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<take_off>)))
  "Returns full string definition for message of type '<take_off>"
  (cl:format cl:nil "int8 drone_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'take_off)))
  "Returns full string definition for message of type 'take_off"
  (cl:format cl:nil "int8 drone_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <take_off>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <take_off>))
  "Converts a ROS message object to a list"
  (cl:list 'take_off
    (cl:cons ':drone_id (drone_id msg))
))
