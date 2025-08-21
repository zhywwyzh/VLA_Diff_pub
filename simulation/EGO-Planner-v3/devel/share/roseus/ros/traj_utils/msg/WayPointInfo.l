;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::WayPointInfo)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'WayPointInfo (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::WAYPOINTINFO")
  (make-package "TRAJ_UTILS::WAYPOINTINFO"))

(in-package "ROS")
;;//! \htmlinclude WayPointInfo.msg.html


(defclass traj_utils::WayPointInfo
  :super ros::object
  :slots (_drone_id _waypoint_id _waypoint_pos ))

(defmethod traj_utils::WayPointInfo
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:waypoint_id __waypoint_id) 0)
    ((:waypoint_pos __waypoint_pos) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _waypoint_id (round __waypoint_id))
   (setq _waypoint_pos __waypoint_pos)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:waypoint_id
   (&optional __waypoint_id)
   (if __waypoint_id (setq _waypoint_id __waypoint_id)) _waypoint_id)
  (:waypoint_pos
   (&optional __waypoint_pos)
   (if __waypoint_pos (setq _waypoint_pos __waypoint_pos)) _waypoint_pos)
  (:serialization-length
   ()
   (+
    ;; int16 _drone_id
    2
    ;; int16 _waypoint_id
    2
    ;; float32[3] _waypoint_pos
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _drone_id
       (write-word _drone_id s)
     ;; int16 _waypoint_id
       (write-word _waypoint_id s)
     ;; float32[3] _waypoint_pos
     (dotimes (i 3)
       (sys::poke (elt _waypoint_pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _drone_id
     (setq _drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _waypoint_id
     (setq _waypoint_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[3] _waypoint_pos
   (dotimes (i (length _waypoint_pos))
     (setf (elt _waypoint_pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get traj_utils::WayPointInfo :md5sum-) "f3e1a39e25f76f15df8b36b8cc49d0af")
(setf (get traj_utils::WayPointInfo :datatype-) "traj_utils/WayPointInfo")
(setf (get traj_utils::WayPointInfo :definition-)
      "int16 drone_id
int16 waypoint_id
float32[3] waypoint_pos
")



(provide :traj_utils/WayPointInfo "f3e1a39e25f76f15df8b36b8cc49d0af")


