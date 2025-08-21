; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude EgoPlannerResult.msg.html

(cl:defclass <EgoPlannerResult> (roslisp-msg-protocol:ros-message)
  ((planner_goal
    :reader planner_goal
    :initarg :planner_goal
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (plan_times
    :reader plan_times
    :initarg :plan_times
    :type cl:fixnum
    :initform 0)
   (plan_status
    :reader plan_status
    :initarg :plan_status
    :type cl:boolean
    :initform cl:nil)
   (modify_status
    :reader modify_status
    :initarg :modify_status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EgoPlannerResult (<EgoPlannerResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EgoPlannerResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EgoPlannerResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<EgoPlannerResult> is deprecated: use quadrotor_msgs-msg:EgoPlannerResult instead.")))

(cl:ensure-generic-function 'planner_goal-val :lambda-list '(m))
(cl:defmethod planner_goal-val ((m <EgoPlannerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:planner_goal-val is deprecated.  Use quadrotor_msgs-msg:planner_goal instead.")
  (planner_goal m))

(cl:ensure-generic-function 'plan_times-val :lambda-list '(m))
(cl:defmethod plan_times-val ((m <EgoPlannerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:plan_times-val is deprecated.  Use quadrotor_msgs-msg:plan_times instead.")
  (plan_times m))

(cl:ensure-generic-function 'plan_status-val :lambda-list '(m))
(cl:defmethod plan_status-val ((m <EgoPlannerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:plan_status-val is deprecated.  Use quadrotor_msgs-msg:plan_status instead.")
  (plan_status m))

(cl:ensure-generic-function 'modify_status-val :lambda-list '(m))
(cl:defmethod modify_status-val ((m <EgoPlannerResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:modify_status-val is deprecated.  Use quadrotor_msgs-msg:modify_status instead.")
  (modify_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EgoPlannerResult>) ostream)
  "Serializes a message object of type '<EgoPlannerResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'planner_goal) ostream)
  (cl:let* ((signed (cl:slot-value msg 'plan_times)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'plan_status) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'modify_status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EgoPlannerResult>) istream)
  "Deserializes a message object of type '<EgoPlannerResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'planner_goal) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plan_times) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'plan_status) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'modify_status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EgoPlannerResult>)))
  "Returns string type for a message object of type '<EgoPlannerResult>"
  "quadrotor_msgs/EgoPlannerResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EgoPlannerResult)))
  "Returns string type for a message object of type 'EgoPlannerResult"
  "quadrotor_msgs/EgoPlannerResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EgoPlannerResult>)))
  "Returns md5sum for a message object of type '<EgoPlannerResult>"
  "e6cf40d72e8bfa9282deb6b127bfe25d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EgoPlannerResult)))
  "Returns md5sum for a message object of type 'EgoPlannerResult"
  "e6cf40d72e8bfa9282deb6b127bfe25d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EgoPlannerResult>)))
  "Returns full string definition for message of type '<EgoPlannerResult>"
  (cl:format cl:nil "#data structure~%geometry_msgs/Vector3 planner_goal~%int16 plan_times~%bool plan_status~%bool modify_status~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EgoPlannerResult)))
  "Returns full string definition for message of type 'EgoPlannerResult"
  (cl:format cl:nil "#data structure~%geometry_msgs/Vector3 planner_goal~%int16 plan_times~%bool plan_status~%bool modify_status~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EgoPlannerResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'planner_goal))
     2
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EgoPlannerResult>))
  "Converts a ROS message object to a list"
  (cl:list 'EgoPlannerResult
    (cl:cons ':planner_goal (planner_goal msg))
    (cl:cons ':plan_times (plan_times msg))
    (cl:cons ':plan_status (plan_status msg))
    (cl:cons ':modify_status (modify_status msg))
))
