; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude StatusData.msg.html

(cl:defclass <StatusData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (loop_rate
    :reader loop_rate
    :initarg :loop_rate
    :type cl:fixnum
    :initform 0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (seq
    :reader seq
    :initarg :seq
    :type cl:fixnum
    :initform 0)
   (dead
    :reader dead
    :initarg :dead
    :type cl:fixnum
    :initform 0)
   (dead_pos
    :reader dead_pos
    :initarg :dead_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (dead_vel
    :reader dead_vel
    :initarg :dead_vel
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass StatusData (<StatusData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StatusData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StatusData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<StatusData> is deprecated: use traj_utils-msg:StatusData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:header-val is deprecated.  Use traj_utils-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'loop_rate-val :lambda-list '(m))
(cl:defmethod loop_rate-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:loop_rate-val is deprecated.  Use traj_utils-msg:loop_rate instead.")
  (loop_rate m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:voltage-val is deprecated.  Use traj_utils-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:seq-val is deprecated.  Use traj_utils-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'dead-val :lambda-list '(m))
(cl:defmethod dead-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:dead-val is deprecated.  Use traj_utils-msg:dead instead.")
  (dead m))

(cl:ensure-generic-function 'dead_pos-val :lambda-list '(m))
(cl:defmethod dead_pos-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:dead_pos-val is deprecated.  Use traj_utils-msg:dead_pos instead.")
  (dead_pos m))

(cl:ensure-generic-function 'dead_vel-val :lambda-list '(m))
(cl:defmethod dead_vel-val ((m <StatusData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:dead_vel-val is deprecated.  Use traj_utils-msg:dead_vel instead.")
  (dead_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StatusData>) ostream)
  "Serializes a message object of type '<StatusData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drone_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_rate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_rate)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dead)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dead_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dead_vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StatusData>) istream)
  "Deserializes a message object of type '<StatusData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drone_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'loop_rate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'loop_rate)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dead)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dead_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dead_vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StatusData>)))
  "Returns string type for a message object of type '<StatusData>"
  "traj_utils/StatusData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StatusData)))
  "Returns string type for a message object of type 'StatusData"
  "traj_utils/StatusData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StatusData>)))
  "Returns md5sum for a message object of type '<StatusData>"
  "fb7b2c8a913bcd9ee73aff94ba73e8f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StatusData)))
  "Returns md5sum for a message object of type 'StatusData"
  "fb7b2c8a913bcd9ee73aff94ba73e8f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StatusData>)))
  "Returns full string definition for message of type '<StatusData>"
  (cl:format cl:nil "Header header~%uint8 drone_id~%uint16 loop_rate~%float64 voltage~%uint8 seq~%uint8 dead~%geometry_msgs/Point dead_pos~%geometry_msgs/Point dead_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StatusData)))
  "Returns full string definition for message of type 'StatusData"
  (cl:format cl:nil "Header header~%uint8 drone_id~%uint16 loop_rate~%float64 voltage~%uint8 seq~%uint8 dead~%geometry_msgs/Point dead_pos~%geometry_msgs/Point dead_vel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StatusData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     8
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dead_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dead_vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StatusData>))
  "Converts a ROS message object to a list"
  (cl:list 'StatusData
    (cl:cons ':header (header msg))
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':loop_rate (loop_rate msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':seq (seq msg))
    (cl:cons ':dead (dead msg))
    (cl:cons ':dead_pos (dead_pos msg))
    (cl:cons ':dead_vel (dead_vel msg))
))
