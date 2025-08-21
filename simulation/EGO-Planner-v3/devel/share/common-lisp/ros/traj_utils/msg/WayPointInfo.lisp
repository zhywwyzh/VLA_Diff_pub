; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude WayPointInfo.msg.html

(cl:defclass <WayPointInfo> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:fixnum
    :initform 0)
   (waypoint_id
    :reader waypoint_id
    :initarg :waypoint_id
    :type cl:fixnum
    :initform 0)
   (waypoint_pos
    :reader waypoint_pos
    :initarg :waypoint_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass WayPointInfo (<WayPointInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WayPointInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WayPointInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<WayPointInfo> is deprecated: use traj_utils-msg:WayPointInfo instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <WayPointInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_id-val is deprecated.  Use traj_utils-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'waypoint_id-val :lambda-list '(m))
(cl:defmethod waypoint_id-val ((m <WayPointInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:waypoint_id-val is deprecated.  Use traj_utils-msg:waypoint_id instead.")
  (waypoint_id m))

(cl:ensure-generic-function 'waypoint_pos-val :lambda-list '(m))
(cl:defmethod waypoint_pos-val ((m <WayPointInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:waypoint_pos-val is deprecated.  Use traj_utils-msg:waypoint_pos instead.")
  (waypoint_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WayPointInfo>) ostream)
  "Serializes a message object of type '<WayPointInfo>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'waypoint_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waypoint_pos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WayPointInfo>) istream)
  "Deserializes a message object of type '<WayPointInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'waypoint_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:setf (cl:slot-value msg 'waypoint_pos) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'waypoint_pos)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WayPointInfo>)))
  "Returns string type for a message object of type '<WayPointInfo>"
  "traj_utils/WayPointInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WayPointInfo)))
  "Returns string type for a message object of type 'WayPointInfo"
  "traj_utils/WayPointInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WayPointInfo>)))
  "Returns md5sum for a message object of type '<WayPointInfo>"
  "f3e1a39e25f76f15df8b36b8cc49d0af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WayPointInfo)))
  "Returns md5sum for a message object of type 'WayPointInfo"
  "f3e1a39e25f76f15df8b36b8cc49d0af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WayPointInfo>)))
  "Returns full string definition for message of type '<WayPointInfo>"
  (cl:format cl:nil "int16 drone_id~%int16 waypoint_id~%float32[3] waypoint_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WayPointInfo)))
  "Returns full string definition for message of type 'WayPointInfo"
  (cl:format cl:nil "int16 drone_id~%int16 waypoint_id~%float32[3] waypoint_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WayPointInfo>))
  (cl:+ 0
     2
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoint_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WayPointInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'WayPointInfo
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':waypoint_id (waypoint_id msg))
    (cl:cons ':waypoint_pos (waypoint_pos msg))
))
