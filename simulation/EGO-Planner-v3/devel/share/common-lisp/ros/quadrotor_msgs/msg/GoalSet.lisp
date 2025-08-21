; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude GoalSet.msg.html

(cl:defclass <GoalSet> (roslisp-msg-protocol:ros-message)
  ((to_drone_ids
    :reader to_drone_ids
    :initarg :to_drone_ids
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (goal
    :reader goal
    :initarg :goal
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (yaw
    :reader yaw
    :initarg :yaw
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (look_forward
    :reader look_forward
    :initarg :look_forward
    :type cl:boolean
    :initform cl:nil)
   (goal_to_follower
    :reader goal_to_follower
    :initarg :goal_to_follower
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GoalSet (<GoalSet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoalSet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoalSet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<GoalSet> is deprecated: use quadrotor_msgs-msg:GoalSet instead.")))

(cl:ensure-generic-function 'to_drone_ids-val :lambda-list '(m))
(cl:defmethod to_drone_ids-val ((m <GoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:to_drone_ids-val is deprecated.  Use quadrotor_msgs-msg:to_drone_ids instead.")
  (to_drone_ids m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <GoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:goal-val is deprecated.  Use quadrotor_msgs-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <GoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:yaw-val is deprecated.  Use quadrotor_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'look_forward-val :lambda-list '(m))
(cl:defmethod look_forward-val ((m <GoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:look_forward-val is deprecated.  Use quadrotor_msgs-msg:look_forward instead.")
  (look_forward m))

(cl:ensure-generic-function 'goal_to_follower-val :lambda-list '(m))
(cl:defmethod goal_to_follower-val ((m <GoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:goal_to_follower-val is deprecated.  Use quadrotor_msgs-msg:goal_to_follower instead.")
  (goal_to_follower m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoalSet>) ostream)
  "Serializes a message object of type '<GoalSet>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'to_drone_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'to_drone_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'goal))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'yaw))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'look_forward) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'goal_to_follower) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoalSet>) istream)
  "Deserializes a message object of type '<GoalSet>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'to_drone_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'to_drone_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yaw) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'yaw)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'look_forward) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'goal_to_follower) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoalSet>)))
  "Returns string type for a message object of type '<GoalSet>"
  "quadrotor_msgs/GoalSet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoalSet)))
  "Returns string type for a message object of type 'GoalSet"
  "quadrotor_msgs/GoalSet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoalSet>)))
  "Returns md5sum for a message object of type '<GoalSet>"
  "fce849fd3f9e593aecb0eb34b6f685a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoalSet)))
  "Returns md5sum for a message object of type 'GoalSet"
  "fce849fd3f9e593aecb0eb34b6f685a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoalSet>)))
  "Returns full string definition for message of type '<GoalSet>"
  (cl:format cl:nil "uint8[]               to_drone_ids~%geometry_msgs/Point[] goal~%float32[]             yaw~%bool                  look_forward~%bool                  goal_to_follower~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoalSet)))
  "Returns full string definition for message of type 'GoalSet"
  (cl:format cl:nil "uint8[]               to_drone_ids~%geometry_msgs/Point[] goal~%float32[]             yaw~%bool                  look_forward~%bool                  goal_to_follower~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoalSet>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'to_drone_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'yaw) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoalSet>))
  "Converts a ROS message object to a list"
  (cl:list 'GoalSet
    (cl:cons ':to_drone_ids (to_drone_ids msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':look_forward (look_forward msg))
    (cl:cons ':goal_to_follower (goal_to_follower msg))
))
