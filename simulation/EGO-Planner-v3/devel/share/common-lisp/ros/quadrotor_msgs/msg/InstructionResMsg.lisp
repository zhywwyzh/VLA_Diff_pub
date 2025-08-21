; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude InstructionResMsg.msg.html

(cl:defclass <InstructionResMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (instruction_type
    :reader instruction_type
    :initarg :instruction_type
    :type cl:fixnum
    :initform 0)
   (drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (is_succeed
    :reader is_succeed
    :initarg :is_succeed
    :type cl:boolean
    :initform cl:nil)
   (tar_drone_id
    :reader tar_drone_id
    :initarg :tar_drone_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (src_drone_id
    :reader src_drone_id
    :initarg :src_drone_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (failed_drone_id
    :reader failed_drone_id
    :initarg :failed_drone_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass InstructionResMsg (<InstructionResMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InstructionResMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InstructionResMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<InstructionResMsg> is deprecated: use quadrotor_msgs-msg:InstructionResMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <InstructionResMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'instruction_type-val :lambda-list '(m))
(cl:defmethod instruction_type-val ((m <InstructionResMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:instruction_type-val is deprecated.  Use quadrotor_msgs-msg:instruction_type instead.")
  (instruction_type m))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <InstructionResMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:drone_id-val is deprecated.  Use quadrotor_msgs-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'is_succeed-val :lambda-list '(m))
(cl:defmethod is_succeed-val ((m <InstructionResMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:is_succeed-val is deprecated.  Use quadrotor_msgs-msg:is_succeed instead.")
  (is_succeed m))

(cl:ensure-generic-function 'tar_drone_id-val :lambda-list '(m))
(cl:defmethod tar_drone_id-val ((m <InstructionResMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tar_drone_id-val is deprecated.  Use quadrotor_msgs-msg:tar_drone_id instead.")
  (tar_drone_id m))

(cl:ensure-generic-function 'src_drone_id-val :lambda-list '(m))
(cl:defmethod src_drone_id-val ((m <InstructionResMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:src_drone_id-val is deprecated.  Use quadrotor_msgs-msg:src_drone_id instead.")
  (src_drone_id m))

(cl:ensure-generic-function 'failed_drone_id-val :lambda-list '(m))
(cl:defmethod failed_drone_id-val ((m <InstructionResMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:failed_drone_id-val is deprecated.  Use quadrotor_msgs-msg:failed_drone_id instead.")
  (failed_drone_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<InstructionResMsg>)))
    "Constants for message type '<InstructionResMsg>"
  '((:TURN_GOAL . 1)
    (:TURN_EXPLORE . 2)
    (:TURN_PATROL . 3)
    (:TURN_HIT . 4)
    (:GO_HOME . 5)
    (:TURN_EGO_GOAL . 6)
    (:SHARE_MAP . 7)
    (:RESET_EXPLORE_AREA . 8)
    (:MAP_MERGE_REQUEST . 9)
    (:MAP_CIRCULATE . 10))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'InstructionResMsg)))
    "Constants for message type 'InstructionResMsg"
  '((:TURN_GOAL . 1)
    (:TURN_EXPLORE . 2)
    (:TURN_PATROL . 3)
    (:TURN_HIT . 4)
    (:GO_HOME . 5)
    (:TURN_EGO_GOAL . 6)
    (:SHARE_MAP . 7)
    (:RESET_EXPLORE_AREA . 8)
    (:MAP_MERGE_REQUEST . 9)
    (:MAP_CIRCULATE . 10))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InstructionResMsg>) ostream)
  "Serializes a message object of type '<InstructionResMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'instruction_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drone_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_succeed) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tar_drone_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'tar_drone_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src_drone_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'src_drone_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'failed_drone_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'failed_drone_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InstructionResMsg>) istream)
  "Deserializes a message object of type '<InstructionResMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'instruction_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drone_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'is_succeed) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tar_drone_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tar_drone_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src_drone_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src_drone_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'failed_drone_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'failed_drone_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InstructionResMsg>)))
  "Returns string type for a message object of type '<InstructionResMsg>"
  "quadrotor_msgs/InstructionResMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InstructionResMsg)))
  "Returns string type for a message object of type 'InstructionResMsg"
  "quadrotor_msgs/InstructionResMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InstructionResMsg>)))
  "Returns md5sum for a message object of type '<InstructionResMsg>"
  "ebdd566ae6eb6d7e30fd1625d7dfda91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InstructionResMsg)))
  "Returns md5sum for a message object of type 'InstructionResMsg"
  "ebdd566ae6eb6d7e30fd1625d7dfda91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InstructionResMsg>)))
  "Returns full string definition for message of type '<InstructionResMsg>"
  (cl:format cl:nil "Header header~%uint8 instruction_type~%uint8 drone_id~%bool is_succeed~%~%# -- map merge and share map -- #~%uint8[] tar_drone_id~%uint8[] src_drone_id~%uint8[] failed_drone_id~%~%# -- instruction type defination-- #~%uint8 TURN_GOAL = 1~%uint8 TURN_EXPLORE = 2~%uint8 TURN_PATROL = 3~%uint8 TURN_HIT = 4~%uint8 GO_HOME = 5~%uint8 TURN_EGO_GOAL = 6~%uint8 SHARE_MAP = 7~%uint8 RESET_EXPLORE_AREA = 8~%uint8 MAP_MERGE_REQUEST = 9~%uint8 MAP_CIRCULATE = 10~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InstructionResMsg)))
  "Returns full string definition for message of type 'InstructionResMsg"
  (cl:format cl:nil "Header header~%uint8 instruction_type~%uint8 drone_id~%bool is_succeed~%~%# -- map merge and share map -- #~%uint8[] tar_drone_id~%uint8[] src_drone_id~%uint8[] failed_drone_id~%~%# -- instruction type defination-- #~%uint8 TURN_GOAL = 1~%uint8 TURN_EXPLORE = 2~%uint8 TURN_PATROL = 3~%uint8 TURN_HIT = 4~%uint8 GO_HOME = 5~%uint8 TURN_EGO_GOAL = 6~%uint8 SHARE_MAP = 7~%uint8 RESET_EXPLORE_AREA = 8~%uint8 MAP_MERGE_REQUEST = 9~%uint8 MAP_CIRCULATE = 10~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InstructionResMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tar_drone_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src_drone_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'failed_drone_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InstructionResMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'InstructionResMsg
    (cl:cons ':header (header msg))
    (cl:cons ':instruction_type (instruction_type msg))
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':is_succeed (is_succeed msg))
    (cl:cons ':tar_drone_id (tar_drone_id msg))
    (cl:cons ':src_drone_id (src_drone_id msg))
    (cl:cons ':failed_drone_id (failed_drone_id msg))
))
