;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::StatusData)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'StatusData (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::STATUSDATA")
  (make-package "TRAJ_UTILS::STATUSDATA"))

(in-package "ROS")
;;//! \htmlinclude StatusData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass traj_utils::StatusData
  :super ros::object
  :slots (_header _drone_id _loop_rate _voltage _seq _dead _dead_pos _dead_vel ))

(defmethod traj_utils::StatusData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drone_id __drone_id) 0)
    ((:loop_rate __loop_rate) 0)
    ((:voltage __voltage) 0.0)
    ((:seq __seq) 0)
    ((:dead __dead) 0)
    ((:dead_pos __dead_pos) (instance geometry_msgs::Point :init))
    ((:dead_vel __dead_vel) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _drone_id (round __drone_id))
   (setq _loop_rate (round __loop_rate))
   (setq _voltage (float __voltage))
   (setq _seq (round __seq))
   (setq _dead (round __dead))
   (setq _dead_pos __dead_pos)
   (setq _dead_vel __dead_vel)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:loop_rate
   (&optional __loop_rate)
   (if __loop_rate (setq _loop_rate __loop_rate)) _loop_rate)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:seq
   (&optional __seq)
   (if __seq (setq _seq __seq)) _seq)
  (:dead
   (&optional __dead)
   (if __dead (setq _dead __dead)) _dead)
  (:dead_pos
   (&rest __dead_pos)
   (if (keywordp (car __dead_pos))
       (send* _dead_pos __dead_pos)
     (progn
       (if __dead_pos (setq _dead_pos (car __dead_pos)))
       _dead_pos)))
  (:dead_vel
   (&rest __dead_vel)
   (if (keywordp (car __dead_vel))
       (send* _dead_vel __dead_vel)
     (progn
       (if __dead_vel (setq _dead_vel (car __dead_vel)))
       _dead_vel)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _drone_id
    1
    ;; uint16 _loop_rate
    2
    ;; float64 _voltage
    8
    ;; uint8 _seq
    1
    ;; uint8 _dead
    1
    ;; geometry_msgs/Point _dead_pos
    (send _dead_pos :serialization-length)
    ;; geometry_msgs/Point _dead_vel
    (send _dead_vel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _drone_id
       (write-byte _drone_id s)
     ;; uint16 _loop_rate
       (write-word _loop_rate s)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _seq
       (write-byte _seq s)
     ;; uint8 _dead
       (write-byte _dead s)
     ;; geometry_msgs/Point _dead_pos
       (send _dead_pos :serialize s)
     ;; geometry_msgs/Point _dead_vel
       (send _dead_vel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _drone_id
     (setq _drone_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _loop_rate
     (setq _loop_rate (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _seq
     (setq _seq (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _dead
     (setq _dead (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point _dead_pos
     (send _dead_pos :deserialize buf ptr-) (incf ptr- (send _dead_pos :serialization-length))
   ;; geometry_msgs/Point _dead_vel
     (send _dead_vel :deserialize buf ptr-) (incf ptr- (send _dead_vel :serialization-length))
   ;;
   self)
  )

(setf (get traj_utils::StatusData :md5sum-) "fb7b2c8a913bcd9ee73aff94ba73e8f1")
(setf (get traj_utils::StatusData :datatype-) "traj_utils/StatusData")
(setf (get traj_utils::StatusData :definition-)
      "Header header
uint8 drone_id
uint16 loop_rate
float64 voltage
uint8 seq
uint8 dead
geometry_msgs/Point dead_pos
geometry_msgs/Point dead_vel

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



(provide :traj_utils/StatusData "fb7b2c8a913bcd9ee73aff94ba73e8f1")


