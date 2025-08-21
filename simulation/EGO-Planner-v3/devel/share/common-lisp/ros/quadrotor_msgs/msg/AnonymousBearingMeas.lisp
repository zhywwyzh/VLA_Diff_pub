; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude AnonymousBearingMeas.msg.html

(cl:defclass <AnonymousBearingMeas> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (anonymous_bearing
    :reader anonymous_bearing
    :initarg :anonymous_bearing
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass AnonymousBearingMeas (<AnonymousBearingMeas>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnonymousBearingMeas>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnonymousBearingMeas)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<AnonymousBearingMeas> is deprecated: use quadrotor_msgs-msg:AnonymousBearingMeas instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AnonymousBearingMeas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:id-val is deprecated.  Use quadrotor_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'anonymous_bearing-val :lambda-list '(m))
(cl:defmethod anonymous_bearing-val ((m <AnonymousBearingMeas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:anonymous_bearing-val is deprecated.  Use quadrotor_msgs-msg:anonymous_bearing instead.")
  (anonymous_bearing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnonymousBearingMeas>) ostream)
  "Serializes a message object of type '<AnonymousBearingMeas>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'anonymous_bearing) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnonymousBearingMeas>) istream)
  "Deserializes a message object of type '<AnonymousBearingMeas>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'anonymous_bearing) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnonymousBearingMeas>)))
  "Returns string type for a message object of type '<AnonymousBearingMeas>"
  "quadrotor_msgs/AnonymousBearingMeas")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnonymousBearingMeas)))
  "Returns string type for a message object of type 'AnonymousBearingMeas"
  "quadrotor_msgs/AnonymousBearingMeas")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnonymousBearingMeas>)))
  "Returns md5sum for a message object of type '<AnonymousBearingMeas>"
  "df646f67fec5bf09f38afc544e61637c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnonymousBearingMeas)))
  "Returns md5sum for a message object of type 'AnonymousBearingMeas"
  "df646f67fec5bf09f38afc544e61637c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnonymousBearingMeas>)))
  "Returns full string definition for message of type '<AnonymousBearingMeas>"
  (cl:format cl:nil "uint8 id~%geometry_msgs/PointStamped anonymous_bearing~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnonymousBearingMeas)))
  "Returns full string definition for message of type 'AnonymousBearingMeas"
  (cl:format cl:nil "uint8 id~%geometry_msgs/PointStamped anonymous_bearing~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnonymousBearingMeas>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'anonymous_bearing))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnonymousBearingMeas>))
  "Converts a ROS message object to a list"
  (cl:list 'AnonymousBearingMeas
    (cl:cons ':id (id msg))
    (cl:cons ':anonymous_bearing (anonymous_bearing msg))
))
