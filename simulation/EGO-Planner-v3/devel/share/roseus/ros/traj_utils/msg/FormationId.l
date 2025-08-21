;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::FormationId)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'FormationId (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::FORMATIONID")
  (make-package "TRAJ_UTILS::FORMATIONID"))

(in-package "ROS")
;;//! \htmlinclude FormationId.msg.html


(defclass traj_utils::FormationId
  :super ros::object
  :slots (_drone_formation_id _car_formation_id ))

(defmethod traj_utils::FormationId
  (:init
   (&key
    ((:drone_formation_id __drone_formation_id) 0)
    ((:car_formation_id __car_formation_id) 0)
    )
   (send-super :init)
   (setq _drone_formation_id (round __drone_formation_id))
   (setq _car_formation_id (round __car_formation_id))
   self)
  (:drone_formation_id
   (&optional __drone_formation_id)
   (if __drone_formation_id (setq _drone_formation_id __drone_formation_id)) _drone_formation_id)
  (:car_formation_id
   (&optional __car_formation_id)
   (if __car_formation_id (setq _car_formation_id __car_formation_id)) _car_formation_id)
  (:serialization-length
   ()
   (+
    ;; int16 _drone_formation_id
    2
    ;; int16 _car_formation_id
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _drone_formation_id
       (write-word _drone_formation_id s)
     ;; int16 _car_formation_id
       (write-word _car_formation_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _drone_formation_id
     (setq _drone_formation_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _car_formation_id
     (setq _car_formation_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get traj_utils::FormationId :md5sum-) "b546e1d2050d5e3211b5e5e45523d88e")
(setf (get traj_utils::FormationId :datatype-) "traj_utils/FormationId")
(setf (get traj_utils::FormationId :definition-)
      "int16 drone_formation_id
int16 car_formation_id
")



(provide :traj_utils/FormationId "b546e1d2050d5e3211b5e5e45523d88e")


