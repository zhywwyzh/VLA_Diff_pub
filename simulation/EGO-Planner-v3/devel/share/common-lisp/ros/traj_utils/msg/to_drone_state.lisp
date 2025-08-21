; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude to_drone_state.msg.html

(cl:defclass <to_drone_state> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (current_node_state
    :reader current_node_state
    :initarg :current_node_state
    :type cl:fixnum
    :initform 0)
   (debug_info
    :reader debug_info
    :initarg :debug_info
    :type cl:string
    :initform ""))
)

(cl:defclass to_drone_state (<to_drone_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <to_drone_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'to_drone_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<to_drone_state> is deprecated: use traj_utils-msg:to_drone_state instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <to_drone_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'current_node_state-val :lambda-list '(m))
(cl:defmethod current_node_state-val ((m <to_drone_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:current_node_state-val is deprecated.  Use traj_utils-msg:current_node_state instead.")
  (current_node_state m))

(cl:ensure-generic-function 'debug_info-val :lambda-list '(m))
(cl:defmethod debug_info-val ((m <to_drone_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:debug_info-val is deprecated.  Use traj_utils-msg:debug_info instead.")
  (debug_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <to_drone_state>) ostream)
  "Serializes a message object of type '<to_drone_state>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_node_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'debug_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'debug_info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <to_drone_state>) istream)
  "Deserializes a message object of type '<to_drone_state>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_node_state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'debug_info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'debug_info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<to_drone_state>)))
  "Returns string type for a message object of type '<to_drone_state>"
  "traj_utils/to_drone_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'to_drone_state)))
  "Returns string type for a message object of type 'to_drone_state"
  "traj_utils/to_drone_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<to_drone_state>)))
  "Returns md5sum for a message object of type '<to_drone_state>"
  "82065b5642df1c1cef531f1d008cd434")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'to_drone_state)))
  "Returns md5sum for a message object of type 'to_drone_state"
  "82065b5642df1c1cef531f1d008cd434")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<to_drone_state>)))
  "Returns full string definition for message of type '<to_drone_state>"
  (cl:format cl:nil "int8 drone_id~%# 4: take_off~%# 5: command~%# 6: land~%int8 current_node_state~%string debug_info~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'to_drone_state)))
  "Returns full string definition for message of type 'to_drone_state"
  (cl:format cl:nil "int8 drone_id~%# 4: take_off~%# 5: command~%# 6: land~%int8 current_node_state~%string debug_info~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <to_drone_state>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'debug_info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <to_drone_state>))
  "Converts a ROS message object to a list"
  (cl:list 'to_drone_state
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':current_node_state (current_node_state msg))
    (cl:cons ':debug_info (debug_info msg))
))
