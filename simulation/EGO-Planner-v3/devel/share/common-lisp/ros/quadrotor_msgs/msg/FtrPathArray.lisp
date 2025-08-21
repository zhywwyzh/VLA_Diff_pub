; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude FtrPathArray.msg.html

(cl:defclass <FtrPathArray> (roslisp-msg-protocol:ros-message)
  ((PathArray
    :reader PathArray
    :initarg :PathArray
    :type (cl:vector quadrotor_msgs-msg:FtrPointArray)
   :initform (cl:make-array 0 :element-type 'quadrotor_msgs-msg:FtrPointArray :initial-element (cl:make-instance 'quadrotor_msgs-msg:FtrPointArray))))
)

(cl:defclass FtrPathArray (<FtrPathArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FtrPathArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FtrPathArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<FtrPathArray> is deprecated: use quadrotor_msgs-msg:FtrPathArray instead.")))

(cl:ensure-generic-function 'PathArray-val :lambda-list '(m))
(cl:defmethod PathArray-val ((m <FtrPathArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:PathArray-val is deprecated.  Use quadrotor_msgs-msg:PathArray instead.")
  (PathArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FtrPathArray>) ostream)
  "Serializes a message object of type '<FtrPathArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PathArray))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'PathArray))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FtrPathArray>) istream)
  "Deserializes a message object of type '<FtrPathArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PathArray) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PathArray)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'quadrotor_msgs-msg:FtrPointArray))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FtrPathArray>)))
  "Returns string type for a message object of type '<FtrPathArray>"
  "quadrotor_msgs/FtrPathArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtrPathArray)))
  "Returns string type for a message object of type 'FtrPathArray"
  "quadrotor_msgs/FtrPathArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FtrPathArray>)))
  "Returns md5sum for a message object of type '<FtrPathArray>"
  "d168064edf658ded54eac8a732d98b8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FtrPathArray)))
  "Returns md5sum for a message object of type 'FtrPathArray"
  "d168064edf658ded54eac8a732d98b8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FtrPathArray>)))
  "Returns full string definition for message of type '<FtrPathArray>"
  (cl:format cl:nil "FtrPointArray[] PathArray~%~%================================================================================~%MSG: quadrotor_msgs/FtrPointArray~%geometry_msgs/Point[] PointArray~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FtrPathArray)))
  "Returns full string definition for message of type 'FtrPathArray"
  (cl:format cl:nil "FtrPointArray[] PathArray~%~%================================================================================~%MSG: quadrotor_msgs/FtrPointArray~%geometry_msgs/Point[] PointArray~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FtrPathArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PathArray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FtrPathArray>))
  "Converts a ROS message object to a list"
  (cl:list 'FtrPathArray
    (cl:cons ':PathArray (PathArray msg))
))
