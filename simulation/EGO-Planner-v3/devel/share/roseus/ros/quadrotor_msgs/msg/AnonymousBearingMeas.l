;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::AnonymousBearingMeas)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'AnonymousBearingMeas (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::ANONYMOUSBEARINGMEAS")
  (make-package "QUADROTOR_MSGS::ANONYMOUSBEARINGMEAS"))

(in-package "ROS")
;;//! \htmlinclude AnonymousBearingMeas.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quadrotor_msgs::AnonymousBearingMeas
  :super ros::object
  :slots (_id _anonymous_bearing ))

(defmethod quadrotor_msgs::AnonymousBearingMeas
  (:init
   (&key
    ((:id __id) 0)
    ((:anonymous_bearing __anonymous_bearing) (instance geometry_msgs::PointStamped :init))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _anonymous_bearing __anonymous_bearing)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:anonymous_bearing
   (&rest __anonymous_bearing)
   (if (keywordp (car __anonymous_bearing))
       (send* _anonymous_bearing __anonymous_bearing)
     (progn
       (if __anonymous_bearing (setq _anonymous_bearing (car __anonymous_bearing)))
       _anonymous_bearing)))
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; geometry_msgs/PointStamped _anonymous_bearing
    (send _anonymous_bearing :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; geometry_msgs/PointStamped _anonymous_bearing
       (send _anonymous_bearing :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/PointStamped _anonymous_bearing
     (send _anonymous_bearing :deserialize buf ptr-) (incf ptr- (send _anonymous_bearing :serialization-length))
   ;;
   self)
  )

(setf (get quadrotor_msgs::AnonymousBearingMeas :md5sum-) "df646f67fec5bf09f38afc544e61637c")
(setf (get quadrotor_msgs::AnonymousBearingMeas :datatype-) "quadrotor_msgs/AnonymousBearingMeas")
(setf (get quadrotor_msgs::AnonymousBearingMeas :definition-)
      "uint8 id
geometry_msgs/PointStamped anonymous_bearing
================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :quadrotor_msgs/AnonymousBearingMeas "df646f67fec5bf09f38afc544e61637c")


