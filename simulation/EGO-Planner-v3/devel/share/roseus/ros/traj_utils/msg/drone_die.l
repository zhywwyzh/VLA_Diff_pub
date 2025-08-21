;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::drone_die)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'drone_die (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::DRONE_DIE")
  (make-package "TRAJ_UTILS::DRONE_DIE"))

(in-package "ROS")
;;//! \htmlinclude drone_die.msg.html


(defclass traj_utils::drone_die
  :super ros::object
  :slots (_drone_die ))

(defmethod traj_utils::drone_die
  (:init
   (&key
    ((:drone_die __drone_die) 0)
    )
   (send-super :init)
   (setq _drone_die (round __drone_die))
   self)
  (:drone_die
   (&optional __drone_die)
   (if __drone_die (setq _drone_die __drone_die)) _drone_die)
  (:serialization-length
   ()
   (+
    ;; int32 _drone_die
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _drone_die
       (write-long _drone_die s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _drone_die
     (setq _drone_die (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get traj_utils::drone_die :md5sum-) "469265ba708544468cfac3f514285ad9")
(setf (get traj_utils::drone_die :datatype-) "traj_utils/drone_die")
(setf (get traj_utils::drone_die :definition-)
      "int32 drone_die
")



(provide :traj_utils/drone_die "469265ba708544468cfac3f514285ad9")


