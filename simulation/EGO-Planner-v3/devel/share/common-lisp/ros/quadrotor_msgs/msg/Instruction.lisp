; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude Instruction.msg.html

(cl:defclass <Instruction> (roslisp-msg-protocol:ros-message)
  ((robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:fixnum
    :initform 0)
   (instruction_type
    :reader instruction_type
    :initarg :instruction_type
    :type cl:fixnum
    :initform 0)
   (target_position
    :reader target_position
    :initarg :target_position
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (src_drone_ids
    :reader src_drone_ids
    :initarg :src_drone_ids
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (tar_drone_ids
    :reader tar_drone_ids
    :initarg :tar_drone_ids
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Instruction (<Instruction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Instruction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Instruction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<Instruction> is deprecated: use quadrotor_msgs-msg:Instruction instead.")))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <Instruction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:robot_id-val is deprecated.  Use quadrotor_msgs-msg:robot_id instead.")
  (robot_id m))

(cl:ensure-generic-function 'instruction_type-val :lambda-list '(m))
(cl:defmethod instruction_type-val ((m <Instruction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:instruction_type-val is deprecated.  Use quadrotor_msgs-msg:instruction_type instead.")
  (instruction_type m))

(cl:ensure-generic-function 'target_position-val :lambda-list '(m))
(cl:defmethod target_position-val ((m <Instruction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:target_position-val is deprecated.  Use quadrotor_msgs-msg:target_position instead.")
  (target_position m))

(cl:ensure-generic-function 'src_drone_ids-val :lambda-list '(m))
(cl:defmethod src_drone_ids-val ((m <Instruction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:src_drone_ids-val is deprecated.  Use quadrotor_msgs-msg:src_drone_ids instead.")
  (src_drone_ids m))

(cl:ensure-generic-function 'tar_drone_ids-val :lambda-list '(m))
(cl:defmethod tar_drone_ids-val ((m <Instruction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:tar_drone_ids-val is deprecated.  Use quadrotor_msgs-msg:tar_drone_ids instead.")
  (tar_drone_ids m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Instruction>)))
    "Constants for message type '<Instruction>"
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
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Instruction)))
    "Constants for message type 'Instruction"
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
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Instruction>) ostream)
  "Serializes a message object of type '<Instruction>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'instruction_type)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'target_position))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'src_drone_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'src_drone_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tar_drone_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'tar_drone_ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Instruction>) istream)
  "Deserializes a message object of type '<Instruction>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'instruction_type)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_position) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'target_position)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'src_drone_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'src_drone_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tar_drone_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tar_drone_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Instruction>)))
  "Returns string type for a message object of type '<Instruction>"
  "quadrotor_msgs/Instruction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Instruction)))
  "Returns string type for a message object of type 'Instruction"
  "quadrotor_msgs/Instruction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Instruction>)))
  "Returns md5sum for a message object of type '<Instruction>"
  "6740b6e4e6245eb3348061673116e30f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Instruction)))
  "Returns md5sum for a message object of type 'Instruction"
  "6740b6e4e6245eb3348061673116e30f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Instruction>)))
  "Returns full string definition for message of type '<Instruction>"
  (cl:format cl:nil "uint8 robot_id~%uint8 instruction_type~%float32[3] target_position~%~%# map merge request~%uint16[] src_drone_ids~%uint16[] tar_drone_ids~%~%#definations for instruction_type~%uint8 TURN_GOAL = 1~%uint8 TURN_EXPLORE = 2~%uint8 TURN_PATROL = 3~%uint8 TURN_HIT = 4~%uint8 GO_HOME = 5~%uint8 TURN_EGO_GOAL = 6~%uint8 SHARE_MAP = 7~%uint8 RESET_EXPLORE_AREA = 8~%uint8 MAP_MERGE_REQUEST = 9~%uint8 MAP_CIRCULATE = 10~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Instruction)))
  "Returns full string definition for message of type 'Instruction"
  (cl:format cl:nil "uint8 robot_id~%uint8 instruction_type~%float32[3] target_position~%~%# map merge request~%uint16[] src_drone_ids~%uint16[] tar_drone_ids~%~%#definations for instruction_type~%uint8 TURN_GOAL = 1~%uint8 TURN_EXPLORE = 2~%uint8 TURN_PATROL = 3~%uint8 TURN_HIT = 4~%uint8 GO_HOME = 5~%uint8 TURN_EGO_GOAL = 6~%uint8 SHARE_MAP = 7~%uint8 RESET_EXPLORE_AREA = 8~%uint8 MAP_MERGE_REQUEST = 9~%uint8 MAP_CIRCULATE = 10~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Instruction>))
  (cl:+ 0
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'target_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'src_drone_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tar_drone_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Instruction>))
  "Converts a ROS message object to a list"
  (cl:list 'Instruction
    (cl:cons ':robot_id (robot_id msg))
    (cl:cons ':instruction_type (instruction_type msg))
    (cl:cons ':target_position (target_position msg))
    (cl:cons ':src_drone_ids (src_drone_ids msg))
    (cl:cons ':tar_drone_ids (tar_drone_ids msg))
))
