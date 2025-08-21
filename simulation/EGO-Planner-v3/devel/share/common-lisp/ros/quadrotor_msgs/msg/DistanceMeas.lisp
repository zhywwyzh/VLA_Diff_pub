; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude DistanceMeas.msg.html

(cl:defclass <DistanceMeas> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (distance_meas
    :reader distance_meas
    :initarg :distance_meas
    :type quadrotor_msgs-msg:LinktrackNodeframe3
    :initform (cl:make-instance 'quadrotor_msgs-msg:LinktrackNodeframe3)))
)

(cl:defclass DistanceMeas (<DistanceMeas>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistanceMeas>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistanceMeas)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<DistanceMeas> is deprecated: use quadrotor_msgs-msg:DistanceMeas instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DistanceMeas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'distance_meas-val :lambda-list '(m))
(cl:defmethod distance_meas-val ((m <DistanceMeas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:distance_meas-val is deprecated.  Use quadrotor_msgs-msg:distance_meas instead.")
  (distance_meas m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistanceMeas>) ostream)
  "Serializes a message object of type '<DistanceMeas>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'distance_meas) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistanceMeas>) istream)
  "Deserializes a message object of type '<DistanceMeas>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'distance_meas) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistanceMeas>)))
  "Returns string type for a message object of type '<DistanceMeas>"
  "quadrotor_msgs/DistanceMeas")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistanceMeas)))
  "Returns string type for a message object of type 'DistanceMeas"
  "quadrotor_msgs/DistanceMeas")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistanceMeas>)))
  "Returns md5sum for a message object of type '<DistanceMeas>"
  "0c3b6590cda39f0c3a802590fc69840f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistanceMeas)))
  "Returns md5sum for a message object of type 'DistanceMeas"
  "0c3b6590cda39f0c3a802590fc69840f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistanceMeas>)))
  "Returns full string definition for message of type '<DistanceMeas>"
  (cl:format cl:nil "std_msgs/Header header~%quadrotor_msgs/LinktrackNodeframe3 distance_meas~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/LinktrackNodeframe3~%uint8 role~%uint8 id~%uint32 local_time~%uint32 system_time~%float32 voltage~%LinktrackNode2[] nodes~%~%================================================================================~%MSG: quadrotor_msgs/LinktrackNode2~%uint8 role~%uint8 id~%float32 dis~%float32 fp_rssi~%float32 rx_rssi~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistanceMeas)))
  "Returns full string definition for message of type 'DistanceMeas"
  (cl:format cl:nil "std_msgs/Header header~%quadrotor_msgs/LinktrackNodeframe3 distance_meas~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/LinktrackNodeframe3~%uint8 role~%uint8 id~%uint32 local_time~%uint32 system_time~%float32 voltage~%LinktrackNode2[] nodes~%~%================================================================================~%MSG: quadrotor_msgs/LinktrackNode2~%uint8 role~%uint8 id~%float32 dis~%float32 fp_rssi~%float32 rx_rssi~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistanceMeas>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'distance_meas))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistanceMeas>))
  "Converts a ROS message object to a list"
  (cl:list 'DistanceMeas
    (cl:cons ':header (header msg))
    (cl:cons ':distance_meas (distance_meas msg))
))
