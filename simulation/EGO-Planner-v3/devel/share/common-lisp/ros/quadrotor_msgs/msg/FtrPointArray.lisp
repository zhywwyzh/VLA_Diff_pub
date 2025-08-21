; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude FtrPointArray.msg.html

(cl:defclass <FtrPointArray> (roslisp-msg-protocol:ros-message)
  ((PointArray
    :reader PointArray
    :initarg :PointArray
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass FtrPointArray (<FtrPointArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FtrPointArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FtrPointArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<FtrPointArray> is deprecated: use quadrotor_msgs-msg:FtrPointArray instead.")))

(cl:ensure-generic-function 'PointArray-val :lambda-list '(m))
(cl:defmethod PointArray-val ((m <FtrPointArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:PointArray-val is deprecated.  Use quadrotor_msgs-msg:PointArray instead.")
  (PointArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FtrPointArray>) ostream)
  "Serializes a message object of type '<FtrPointArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PointArray))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'PointArray))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FtrPointArray>) istream)
  "Deserializes a message object of type '<FtrPointArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PointArray) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PointArray)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FtrPointArray>)))
  "Returns string type for a message object of type '<FtrPointArray>"
  "quadrotor_msgs/FtrPointArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FtrPointArray)))
  "Returns string type for a message object of type 'FtrPointArray"
  "quadrotor_msgs/FtrPointArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FtrPointArray>)))
  "Returns md5sum for a message object of type '<FtrPointArray>"
  "b72da7cba93373ecc2fd54ff85f47989")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FtrPointArray)))
  "Returns md5sum for a message object of type 'FtrPointArray"
  "b72da7cba93373ecc2fd54ff85f47989")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FtrPointArray>)))
  "Returns full string definition for message of type '<FtrPointArray>"
  (cl:format cl:nil "geometry_msgs/Point[] PointArray~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FtrPointArray)))
  "Returns full string definition for message of type 'FtrPointArray"
  (cl:format cl:nil "geometry_msgs/Point[] PointArray~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FtrPointArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PointArray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FtrPointArray>))
  "Converts a ROS message object to a list"
  (cl:list 'FtrPointArray
    (cl:cons ':PointArray (PointArray msg))
))
