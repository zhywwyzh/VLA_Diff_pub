;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::SteamdeckInfo)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'SteamdeckInfo (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::STEAMDECKINFO")
  (make-package "TRAJ_UTILS::STEAMDECKINFO"))

(in-package "ROS")
;;//! \htmlinclude SteamdeckInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass traj_utils::SteamdeckInfo
  :super ros::object
  :slots (_traj_start_trigger _formation_ids ))

(defmethod traj_utils::SteamdeckInfo
  (:init
   (&key
    ((:traj_start_trigger __traj_start_trigger) (instance geometry_msgs::PoseStamped :init))
    ((:formation_ids __formation_ids) (instance traj_utils::FormationId :init))
    )
   (send-super :init)
   (setq _traj_start_trigger __traj_start_trigger)
   (setq _formation_ids __formation_ids)
   self)
  (:traj_start_trigger
   (&rest __traj_start_trigger)
   (if (keywordp (car __traj_start_trigger))
       (send* _traj_start_trigger __traj_start_trigger)
     (progn
       (if __traj_start_trigger (setq _traj_start_trigger (car __traj_start_trigger)))
       _traj_start_trigger)))
  (:formation_ids
   (&rest __formation_ids)
   (if (keywordp (car __formation_ids))
       (send* _formation_ids __formation_ids)
     (progn
       (if __formation_ids (setq _formation_ids (car __formation_ids)))
       _formation_ids)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _traj_start_trigger
    (send _traj_start_trigger :serialization-length)
    ;; traj_utils/FormationId _formation_ids
    (send _formation_ids :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _traj_start_trigger
       (send _traj_start_trigger :serialize s)
     ;; traj_utils/FormationId _formation_ids
       (send _formation_ids :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _traj_start_trigger
     (send _traj_start_trigger :deserialize buf ptr-) (incf ptr- (send _traj_start_trigger :serialization-length))
   ;; traj_utils/FormationId _formation_ids
     (send _formation_ids :deserialize buf ptr-) (incf ptr- (send _formation_ids :serialization-length))
   ;;
   self)
  )

(setf (get traj_utils::SteamdeckInfo :md5sum-) "3ed9ffc74ffa217ce2263aeb795f0421")
(setf (get traj_utils::SteamdeckInfo :datatype-) "traj_utils/SteamdeckInfo")
(setf (get traj_utils::SteamdeckInfo :definition-)
      "# command_realted
geometry_msgs/PoseStamped traj_start_trigger
traj_utils/FormationId formation_ids
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

================================================================================
MSG: traj_utils/FormationId
int16 drone_formation_id
int16 car_formation_id
")



(provide :traj_utils/SteamdeckInfo "3ed9ffc74ffa217ce2263aeb795f0421")


