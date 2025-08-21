;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::DistanceMeas)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'DistanceMeas (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::DISTANCEMEAS")
  (make-package "QUADROTOR_MSGS::DISTANCEMEAS"))

(in-package "ROS")
;;//! \htmlinclude DistanceMeas.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::DistanceMeas
  :super ros::object
  :slots (_header _distance_meas ))

(defmethod quadrotor_msgs::DistanceMeas
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:distance_meas __distance_meas) (instance quadrotor_msgs::LinktrackNodeframe3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _distance_meas __distance_meas)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:distance_meas
   (&rest __distance_meas)
   (if (keywordp (car __distance_meas))
       (send* _distance_meas __distance_meas)
     (progn
       (if __distance_meas (setq _distance_meas (car __distance_meas)))
       _distance_meas)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; quadrotor_msgs/LinktrackNodeframe3 _distance_meas
    (send _distance_meas :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; quadrotor_msgs/LinktrackNodeframe3 _distance_meas
       (send _distance_meas :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; quadrotor_msgs/LinktrackNodeframe3 _distance_meas
     (send _distance_meas :deserialize buf ptr-) (incf ptr- (send _distance_meas :serialization-length))
   ;;
   self)
  )

(setf (get quadrotor_msgs::DistanceMeas :md5sum-) "0c3b6590cda39f0c3a802590fc69840f")
(setf (get quadrotor_msgs::DistanceMeas :datatype-) "quadrotor_msgs/DistanceMeas")
(setf (get quadrotor_msgs::DistanceMeas :definition-)
      "std_msgs/Header header
quadrotor_msgs/LinktrackNodeframe3 distance_meas
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
MSG: quadrotor_msgs/LinktrackNodeframe3
uint8 role
uint8 id
uint32 local_time
uint32 system_time
float32 voltage
LinktrackNode2[] nodes

================================================================================
MSG: quadrotor_msgs/LinktrackNode2
uint8 role
uint8 id
float32 dis
float32 fp_rssi
float32 rx_rssi

")



(provide :quadrotor_msgs/DistanceMeas "0c3b6590cda39f0c3a802590fc69840f")


