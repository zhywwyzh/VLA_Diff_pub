; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude EgoGoalSet.msg.html

(cl:defclass <EgoGoalSet> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (goal
    :reader goal
    :initarg :goal
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
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

(cl:defclass EgoGoalSet (<EgoGoalSet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EgoGoalSet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EgoGoalSet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<EgoGoalSet> is deprecated: use quadrotor_msgs-msg:EgoGoalSet instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <EgoGoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:drone_id-val is deprecated.  Use quadrotor_msgs-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <EgoGoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:goal-val is deprecated.  Use quadrotor_msgs-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <EgoGoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:yaw-val is deprecated.  Use quadrotor_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'look_forward-val :lambda-list '(m))
(cl:defmethod look_forward-val ((m <EgoGoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:look_forward-val is deprecated.  Use quadrotor_msgs-msg:look_forward instead.")
  (look_forward m))

(cl:ensure-generic-function 'goal_to_follower-val :lambda-list '(m))
(cl:defmethod goal_to_follower-val ((m <EgoGoalSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:goal_to_follower-val is deprecated.  Use quadrotor_msgs-msg:goal_to_follower instead.")
  (goal_to_follower m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EgoGoalSet>) ostream)
  "Serializes a message object of type '<EgoGoalSet>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drone_id)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'goal))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'look_forward) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'goal_to_follower) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EgoGoalSet>) istream)
  "Deserializes a message object of type '<EgoGoalSet>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drone_id)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'goal)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'look_forward) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'goal_to_follower) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EgoGoalSet>)))
  "Returns string type for a message object of type '<EgoGoalSet>"
  "quadrotor_msgs/EgoGoalSet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EgoGoalSet)))
  "Returns string type for a message object of type 'EgoGoalSet"
  "quadrotor_msgs/EgoGoalSet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EgoGoalSet>)))
  "Returns md5sum for a message object of type '<EgoGoalSet>"
  "4e000e06493c05ae8165574a33ffc993")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EgoGoalSet)))
  "Returns md5sum for a message object of type 'EgoGoalSet"
  "4e000e06493c05ae8165574a33ffc993")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EgoGoalSet>)))
  "Returns full string definition for message of type '<EgoGoalSet>"
  (cl:format cl:nil "uint8      drone_id~%float32[3] goal~%float32    yaw~%bool       look_forward~%bool       goal_to_follower~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EgoGoalSet)))
  "Returns full string definition for message of type 'EgoGoalSet"
  (cl:format cl:nil "uint8      drone_id~%float32[3] goal~%float32    yaw~%bool       look_forward~%bool       goal_to_follower~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EgoGoalSet>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EgoGoalSet>))
  "Converts a ROS message object to a list"
  (cl:list 'EgoGoalSet
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':look_forward (look_forward msg))
    (cl:cons ':goal_to_follower (goal_to_follower msg))
))
