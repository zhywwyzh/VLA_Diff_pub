; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude LocalTime.msg.html

(cl:defclass <LocalTime> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (start_time
    :reader start_time
    :initarg :start_time
    :type cl:real
    :initform 0)
   (no_syc
    :reader no_syc
    :initarg :no_syc
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LocalTime (<LocalTime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalTime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalTime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<LocalTime> is deprecated: use traj_utils-msg:LocalTime instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <LocalTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <LocalTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:start_time-val is deprecated.  Use traj_utils-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'no_syc-val :lambda-list '(m))
(cl:defmethod no_syc-val ((m <LocalTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:no_syc-val is deprecated.  Use traj_utils-msg:no_syc instead.")
  (no_syc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalTime>) ostream)
  "Serializes a message object of type '<LocalTime>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'start_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'start_time) (cl:floor (cl:slot-value msg 'start_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'no_syc) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalTime>) istream)
  "Deserializes a message object of type '<LocalTime>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:slot-value msg 'no_syc) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalTime>)))
  "Returns string type for a message object of type '<LocalTime>"
  "traj_utils/LocalTime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalTime)))
  "Returns string type for a message object of type 'LocalTime"
  "traj_utils/LocalTime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalTime>)))
  "Returns md5sum for a message object of type '<LocalTime>"
  "2012c91cb1df9e8be07a80d63f630a55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalTime)))
  "Returns md5sum for a message object of type 'LocalTime"
  "2012c91cb1df9e8be07a80d63f630a55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalTime>)))
  "Returns full string definition for message of type '<LocalTime>"
  (cl:format cl:nil "int16 drone_id~%time start_time~%bool no_syc~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalTime)))
  "Returns full string definition for message of type 'LocalTime"
  (cl:format cl:nil "int16 drone_id~%time start_time~%bool no_syc~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalTime>))
  (cl:+ 0
     2
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalTime>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalTime
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':no_syc (no_syc msg))
))
