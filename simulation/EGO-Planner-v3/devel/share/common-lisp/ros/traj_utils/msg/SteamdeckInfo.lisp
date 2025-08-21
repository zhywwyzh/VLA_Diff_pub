; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude SteamdeckInfo.msg.html

(cl:defclass <SteamdeckInfo> (roslisp-msg-protocol:ros-message)
  ((traj_start_trigger
    :reader traj_start_trigger
    :initarg :traj_start_trigger
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (formation_ids
    :reader formation_ids
    :initarg :formation_ids
    :type traj_utils-msg:FormationId
    :initform (cl:make-instance 'traj_utils-msg:FormationId)))
)

(cl:defclass SteamdeckInfo (<SteamdeckInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SteamdeckInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SteamdeckInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<SteamdeckInfo> is deprecated: use traj_utils-msg:SteamdeckInfo instead.")))

(cl:ensure-generic-function 'traj_start_trigger-val :lambda-list '(m))
(cl:defmethod traj_start_trigger-val ((m <SteamdeckInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:traj_start_trigger-val is deprecated.  Use traj_utils-msg:traj_start_trigger instead.")
  (traj_start_trigger m))

(cl:ensure-generic-function 'formation_ids-val :lambda-list '(m))
(cl:defmethod formation_ids-val ((m <SteamdeckInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:formation_ids-val is deprecated.  Use traj_utils-msg:formation_ids instead.")
  (formation_ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SteamdeckInfo>) ostream)
  "Serializes a message object of type '<SteamdeckInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'traj_start_trigger) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'formation_ids) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SteamdeckInfo>) istream)
  "Deserializes a message object of type '<SteamdeckInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'traj_start_trigger) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'formation_ids) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SteamdeckInfo>)))
  "Returns string type for a message object of type '<SteamdeckInfo>"
  "traj_utils/SteamdeckInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SteamdeckInfo)))
  "Returns string type for a message object of type 'SteamdeckInfo"
  "traj_utils/SteamdeckInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SteamdeckInfo>)))
  "Returns md5sum for a message object of type '<SteamdeckInfo>"
  "3ed9ffc74ffa217ce2263aeb795f0421")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SteamdeckInfo)))
  "Returns md5sum for a message object of type 'SteamdeckInfo"
  "3ed9ffc74ffa217ce2263aeb795f0421")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SteamdeckInfo>)))
  "Returns full string definition for message of type '<SteamdeckInfo>"
  (cl:format cl:nil "# command_realted~%geometry_msgs/PoseStamped traj_start_trigger~%traj_utils/FormationId formation_ids~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: traj_utils/FormationId~%int16 drone_formation_id~%int16 car_formation_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SteamdeckInfo)))
  "Returns full string definition for message of type 'SteamdeckInfo"
  (cl:format cl:nil "# command_realted~%geometry_msgs/PoseStamped traj_start_trigger~%traj_utils/FormationId formation_ids~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: traj_utils/FormationId~%int16 drone_formation_id~%int16 car_formation_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SteamdeckInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'traj_start_trigger))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'formation_ids))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SteamdeckInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SteamdeckInfo
    (cl:cons ':traj_start_trigger (traj_start_trigger msg))
    (cl:cons ':formation_ids (formation_ids msg))
))
