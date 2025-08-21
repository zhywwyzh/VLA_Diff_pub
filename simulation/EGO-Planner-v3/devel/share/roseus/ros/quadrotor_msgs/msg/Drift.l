;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::Drift)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'Drift (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::DRIFT")
  (make-package "QUADROTOR_MSGS::DRIFT"))

(in-package "ROS")
;;//! \htmlinclude Drift.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quadrotor_msgs::Drift
  :super ros::object
  :slots (_id _drift ))

(defmethod quadrotor_msgs::Drift
  (:init
   (&key
    ((:id __id) 0)
    ((:drift __drift) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _drift __drift)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:drift
   (&rest __drift)
   (if (keywordp (car __drift))
       (send* _drift __drift)
     (progn
       (if __drift (setq _drift (car __drift)))
       _drift)))
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; geometry_msgs/PoseStamped _drift
    (send _drift :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; geometry_msgs/PoseStamped _drift
       (send _drift :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/PoseStamped _drift
     (send _drift :deserialize buf ptr-) (incf ptr- (send _drift :serialization-length))
   ;;
   self)
  )

(setf (get quadrotor_msgs::Drift :md5sum-) "e21fca28d0b53387a08170be5838e563")
(setf (get quadrotor_msgs::Drift :datatype-) "quadrotor_msgs/Drift")
(setf (get quadrotor_msgs::Drift :definition-)
      "uint8 id
geometry_msgs/PoseStamped drift
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :quadrotor_msgs/Drift "e21fca28d0b53387a08170be5838e563")


