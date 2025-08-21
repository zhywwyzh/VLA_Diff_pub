; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude Drift.msg.html

(cl:defclass <Drift> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (drift
    :reader drift
    :initarg :drift
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass Drift (<Drift>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Drift>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Drift)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<Drift> is deprecated: use quadrotor_msgs-msg:Drift instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Drift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:id-val is deprecated.  Use quadrotor_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'drift-val :lambda-list '(m))
(cl:defmethod drift-val ((m <Drift>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:drift-val is deprecated.  Use quadrotor_msgs-msg:drift instead.")
  (drift m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Drift>) ostream)
  "Serializes a message object of type '<Drift>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'drift) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Drift>) istream)
  "Deserializes a message object of type '<Drift>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'drift) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Drift>)))
  "Returns string type for a message object of type '<Drift>"
  "quadrotor_msgs/Drift")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Drift)))
  "Returns string type for a message object of type 'Drift"
  "quadrotor_msgs/Drift")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Drift>)))
  "Returns md5sum for a message object of type '<Drift>"
  "e21fca28d0b53387a08170be5838e563")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Drift)))
  "Returns md5sum for a message object of type 'Drift"
  "e21fca28d0b53387a08170be5838e563")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Drift>)))
  "Returns full string definition for message of type '<Drift>"
  (cl:format cl:nil "uint8 id~%geometry_msgs/PoseStamped drift~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Drift)))
  "Returns full string definition for message of type 'Drift"
  (cl:format cl:nil "uint8 id~%geometry_msgs/PoseStamped drift~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Drift>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'drift))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Drift>))
  "Converts a ROS message object to a list"
  (cl:list 'Drift
    (cl:cons ':id (id msg))
    (cl:cons ':drift (drift msg))
))
