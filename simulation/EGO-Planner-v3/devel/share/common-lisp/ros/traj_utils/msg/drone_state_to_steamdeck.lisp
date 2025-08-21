; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude drone_state_to_steamdeck.msg.html

(cl:defclass <drone_state_to_steamdeck> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (start_time
    :reader start_time
    :initarg :start_time
    :type cl:real
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

(cl:defclass drone_state_to_steamdeck (<drone_state_to_steamdeck>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drone_state_to_steamdeck>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drone_state_to_steamdeck)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<drone_state_to_steamdeck> is deprecated: use traj_utils-msg:drone_state_to_steamdeck instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <drone_state_to_steamdeck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <drone_state_to_steamdeck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:start_time-val is deprecated.  Use traj_utils-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'current_node_state-val :lambda-list '(m))
(cl:defmethod current_node_state-val ((m <drone_state_to_steamdeck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:current_node_state-val is deprecated.  Use traj_utils-msg:current_node_state instead.")
  (current_node_state m))

(cl:ensure-generic-function 'debug_info-val :lambda-list '(m))
(cl:defmethod debug_info-val ((m <drone_state_to_steamdeck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:debug_info-val is deprecated.  Use traj_utils-msg:debug_info instead.")
  (debug_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drone_state_to_steamdeck>) ostream)
  "Serializes a message object of type '<drone_state_to_steamdeck>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'start_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'start_time) (cl:floor (cl:slot-value msg 'start_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drone_state_to_steamdeck>) istream)
  "Deserializes a message object of type '<drone_state_to_steamdeck>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drone_state_to_steamdeck>)))
  "Returns string type for a message object of type '<drone_state_to_steamdeck>"
  "traj_utils/drone_state_to_steamdeck")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drone_state_to_steamdeck)))
  "Returns string type for a message object of type 'drone_state_to_steamdeck"
  "traj_utils/drone_state_to_steamdeck")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drone_state_to_steamdeck>)))
  "Returns md5sum for a message object of type '<drone_state_to_steamdeck>"
  "d799a4895f8a152d4ada9c7313ecec9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drone_state_to_steamdeck)))
  "Returns md5sum for a message object of type 'drone_state_to_steamdeck"
  "d799a4895f8a152d4ada9c7313ecec9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drone_state_to_steamdeck>)))
  "Returns full string definition for message of type '<drone_state_to_steamdeck>"
  (cl:format cl:nil "int8 drone_id~%# 1: baseFail (vins_odom too low or no odom)~%# 2: basepreparing~%# 3: baseReady~%# 4: px4_take_off~%# 5: command~%# 6: px4_land~%# 7: auto_hover~%# 8: locatizationError (optical_flow)~%# 9: crash (cmd ---> stopPropeller) ~%# 10: drone_node_only~%# 11: timeAlignFail~%time start_time~%int8 current_node_state~%string debug_info~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drone_state_to_steamdeck)))
  "Returns full string definition for message of type 'drone_state_to_steamdeck"
  (cl:format cl:nil "int8 drone_id~%# 1: baseFail (vins_odom too low or no odom)~%# 2: basepreparing~%# 3: baseReady~%# 4: px4_take_off~%# 5: command~%# 6: px4_land~%# 7: auto_hover~%# 8: locatizationError (optical_flow)~%# 9: crash (cmd ---> stopPropeller) ~%# 10: drone_node_only~%# 11: timeAlignFail~%time start_time~%int8 current_node_state~%string debug_info~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drone_state_to_steamdeck>))
  (cl:+ 0
     1
     8
     1
     4 (cl:length (cl:slot-value msg 'debug_info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drone_state_to_steamdeck>))
  "Converts a ROS message object to a list"
  (cl:list 'drone_state_to_steamdeck
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':current_node_state (current_node_state msg))
    (cl:cons ':debug_info (debug_info msg))
))
