; Auto-generated. Do not edit!


(cl:in-package traj_utils-msg)


;//! \htmlinclude FormationId.msg.html

(cl:defclass <FormationId> (roslisp-msg-protocol:ros-message)
  ((drone_formation_id
    :reader drone_formation_id
    :initarg :drone_formation_id
    :type cl:fixnum
    :initform 0)
   (car_formation_id
    :reader car_formation_id
    :initarg :car_formation_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FormationId (<FormationId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FormationId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FormationId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_utils-msg:<FormationId> is deprecated: use traj_utils-msg:FormationId instead.")))

(cl:ensure-generic-function 'drone_formation_id-val :lambda-list '(m))
(cl:defmethod drone_formation_id-val ((m <FormationId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:drone_formation_id-val is deprecated.  Use traj_utils-msg:drone_formation_id instead.")
  (drone_formation_id m))

(cl:ensure-generic-function 'car_formation_id-val :lambda-list '(m))
(cl:defmethod car_formation_id-val ((m <FormationId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_utils-msg:car_formation_id-val is deprecated.  Use traj_utils-msg:car_formation_id instead.")
  (car_formation_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FormationId>) ostream)
  "Serializes a message object of type '<FormationId>"
  (cl:let* ((signed (cl:slot-value msg 'drone_formation_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'car_formation_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FormationId>) istream)
  "Deserializes a message object of type '<FormationId>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_formation_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'car_formation_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FormationId>)))
  "Returns string type for a message object of type '<FormationId>"
  "traj_utils/FormationId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FormationId)))
  "Returns string type for a message object of type 'FormationId"
  "traj_utils/FormationId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FormationId>)))
  "Returns md5sum for a message object of type '<FormationId>"
  "b546e1d2050d5e3211b5e5e45523d88e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FormationId)))
  "Returns md5sum for a message object of type 'FormationId"
  "b546e1d2050d5e3211b5e5e45523d88e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FormationId>)))
  "Returns full string definition for message of type '<FormationId>"
  (cl:format cl:nil "int16 drone_formation_id~%int16 car_formation_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FormationId)))
  "Returns full string definition for message of type 'FormationId"
  (cl:format cl:nil "int16 drone_formation_id~%int16 car_formation_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FormationId>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FormationId>))
  "Converts a ROS message object to a list"
  (cl:list 'FormationId
    (cl:cons ':drone_formation_id (drone_formation_id msg))
    (cl:cons ':car_formation_id (car_formation_id msg))
))
