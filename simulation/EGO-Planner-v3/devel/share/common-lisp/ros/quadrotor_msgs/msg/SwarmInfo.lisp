; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude SwarmInfo.msg.html

(cl:defclass <SwarmInfo> (roslisp-msg-protocol:ros-message)
  ((swarm_id
    :reader swarm_id
    :initarg :swarm_id
    :type cl:integer
    :initform 0)
   (robot_ids
    :reader robot_ids
    :initarg :robot_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (leader_id
    :reader leader_id
    :initarg :leader_id
    :type cl:integer
    :initform 0))
)

(cl:defclass SwarmInfo (<SwarmInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwarmInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwarmInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<SwarmInfo> is deprecated: use quadrotor_msgs-msg:SwarmInfo instead.")))

(cl:ensure-generic-function 'swarm_id-val :lambda-list '(m))
(cl:defmethod swarm_id-val ((m <SwarmInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:swarm_id-val is deprecated.  Use quadrotor_msgs-msg:swarm_id instead.")
  (swarm_id m))

(cl:ensure-generic-function 'robot_ids-val :lambda-list '(m))
(cl:defmethod robot_ids-val ((m <SwarmInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:robot_ids-val is deprecated.  Use quadrotor_msgs-msg:robot_ids instead.")
  (robot_ids m))

(cl:ensure-generic-function 'leader_id-val :lambda-list '(m))
(cl:defmethod leader_id-val ((m <SwarmInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:leader_id-val is deprecated.  Use quadrotor_msgs-msg:leader_id instead.")
  (leader_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwarmInfo>) ostream)
  "Serializes a message object of type '<SwarmInfo>"
  (cl:let* ((signed (cl:slot-value msg 'swarm_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robot_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'robot_ids))
  (cl:let* ((signed (cl:slot-value msg 'leader_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwarmInfo>) istream)
  "Deserializes a message object of type '<SwarmInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'swarm_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robot_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robot_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leader_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwarmInfo>)))
  "Returns string type for a message object of type '<SwarmInfo>"
  "quadrotor_msgs/SwarmInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwarmInfo)))
  "Returns string type for a message object of type 'SwarmInfo"
  "quadrotor_msgs/SwarmInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwarmInfo>)))
  "Returns md5sum for a message object of type '<SwarmInfo>"
  "7e58f24709b97dba610bf92bfa971d6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwarmInfo)))
  "Returns md5sum for a message object of type 'SwarmInfo"
  "7e58f24709b97dba610bf92bfa971d6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwarmInfo>)))
  "Returns full string definition for message of type '<SwarmInfo>"
  (cl:format cl:nil "int32 swarm_id~%int32[] robot_ids~%int32 leader_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwarmInfo)))
  "Returns full string definition for message of type 'SwarmInfo"
  (cl:format cl:nil "int32 swarm_id~%int32[] robot_ids~%int32 leader_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwarmInfo>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwarmInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SwarmInfo
    (cl:cons ':swarm_id (swarm_id msg))
    (cl:cons ':robot_ids (robot_ids msg))
    (cl:cons ':leader_id (leader_id msg))
))
