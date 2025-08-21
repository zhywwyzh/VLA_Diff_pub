;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::YawCmd)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'YawCmd (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::YAWCMD")
  (make-package "TRAJ_UTILS::YAWCMD"))

(in-package "ROS")
;;//! \htmlinclude YawCmd.msg.html


(defclass traj_utils::YawCmd
  :super ros::object
  :slots (_drone_id _cmd_id _start_time _cur_yaw _yaw _pos ))

(defmethod traj_utils::YawCmd
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:cmd_id __cmd_id) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:cur_yaw __cur_yaw) 0.0)
    ((:yaw __yaw) 0.0)
    ((:pos __pos) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _cmd_id (round __cmd_id))
   (setq _start_time __start_time)
   (setq _cur_yaw (float __cur_yaw))
   (setq _yaw (float __yaw))
   (setq _pos __pos)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:cmd_id
   (&optional __cmd_id)
   (if __cmd_id (setq _cmd_id __cmd_id)) _cmd_id)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:cur_yaw
   (&optional __cur_yaw)
   (if __cur_yaw (setq _cur_yaw __cur_yaw)) _cur_yaw)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:serialization-length
   ()
   (+
    ;; int16 _drone_id
    2
    ;; int32 _cmd_id
    4
    ;; time _start_time
    8
    ;; float32 _cur_yaw
    4
    ;; float32 _yaw
    4
    ;; float32[3] _pos
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _drone_id
       (write-word _drone_id s)
     ;; int32 _cmd_id
       (write-long _cmd_id s)
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; float32 _cur_yaw
       (sys::poke _cur_yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[3] _pos
     (dotimes (i 3)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _drone_id
     (setq _drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _cmd_id
     (setq _cmd_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _cur_yaw
     (setq _cur_yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[3] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get traj_utils::YawCmd :md5sum-) "21833ee83e61e3e1afedb50075e5a16b")
(setf (get traj_utils::YawCmd :datatype-) "traj_utils/YawCmd")
(setf (get traj_utils::YawCmd :definition-)
      "int16 drone_id
int32 cmd_id
time start_time
float32 cur_yaw
float32 yaw
float32[3] pos 
")



(provide :traj_utils/YawCmd "21833ee83e61e3e1afedb50075e5a16b")


