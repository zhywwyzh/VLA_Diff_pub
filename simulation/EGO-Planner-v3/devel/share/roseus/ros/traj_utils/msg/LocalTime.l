;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::LocalTime)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'LocalTime (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::LOCALTIME")
  (make-package "TRAJ_UTILS::LOCALTIME"))

(in-package "ROS")
;;//! \htmlinclude LocalTime.msg.html


(defclass traj_utils::LocalTime
  :super ros::object
  :slots (_drone_id _start_time _no_syc ))

(defmethod traj_utils::LocalTime
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:no_syc __no_syc) nil)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _start_time __start_time)
   (setq _no_syc __no_syc)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:no_syc
   (&optional (__no_syc :null))
   (if (not (eq __no_syc :null)) (setq _no_syc __no_syc)) _no_syc)
  (:serialization-length
   ()
   (+
    ;; int16 _drone_id
    2
    ;; time _start_time
    8
    ;; bool _no_syc
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _drone_id
       (write-word _drone_id s)
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; bool _no_syc
       (if _no_syc (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _drone_id
     (setq _drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _no_syc
     (setq _no_syc (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get traj_utils::LocalTime :md5sum-) "2012c91cb1df9e8be07a80d63f630a55")
(setf (get traj_utils::LocalTime :datatype-) "traj_utils/LocalTime")
(setf (get traj_utils::LocalTime :definition-)
      "int16 drone_id
time start_time
bool no_syc
")



(provide :traj_utils/LocalTime "2012c91cb1df9e8be07a80d63f630a55")


