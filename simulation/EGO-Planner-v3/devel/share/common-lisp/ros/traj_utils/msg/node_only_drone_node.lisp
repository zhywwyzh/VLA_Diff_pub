; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude node_only_drone_node.msg.html

(cl:defclass <node_only_drone_node> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (node_already_reastart
    :reader node_already_reastart
    :initarg :node_already_reastart
    :type cl:fixnum
    :initform 0))
)

(cl:defclass node_only_drone_node (<node_only_drone_node>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <node_only_drone_node>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'node_only_drone_node)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<node_only_drone_node> is deprecated: use traj_utils-msg:node_only_drone_node instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <node_only_drone_node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'node_already_reastart-val :lambda-list '(m))
(cl:defmethod node_already_reastart-val ((m <node_only_drone_node>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:node_already_reastart-val is deprecated.  Use traj_utils-msg:node_already_reastart instead.")
  (node_already_reastart m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <node_only_drone_node>) ostream)
  "Serializes a message object of type '<node_only_drone_node>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'node_already_reastart)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <node_only_drone_node>) istream)
  "Deserializes a message object of type '<node_only_drone_node>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node_already_reastart) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<node_only_drone_node>)))
  "Returns string type for a message object of type '<node_only_drone_node>"
  "traj_utils/node_only_drone_node")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'node_only_drone_node)))
  "Returns string type for a message object of type 'node_only_drone_node"
  "traj_utils/node_only_drone_node")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<node_only_drone_node>)))
  "Returns md5sum for a message object of type '<node_only_drone_node>"
  "33299fffa442f68d79e075c04bbd062c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'node_only_drone_node)))
  "Returns md5sum for a message object of type 'node_only_drone_node"
  "33299fffa442f68d79e075c04bbd062c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<node_only_drone_node>)))
  "Returns full string definition for message of type '<node_only_drone_node>"
  (cl:format cl:nil "int8 drone_id~%int8 node_already_reastart~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'node_only_drone_node)))
  "Returns full string definition for message of type 'node_only_drone_node"
  (cl:format cl:nil "int8 drone_id~%int8 node_already_reastart~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <node_only_drone_node>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <node_only_drone_node>))
  "Converts a ROS message object to a list"
  (cl:list 'node_only_drone_node
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':node_already_reastart (node_already_reastart msg))
))
