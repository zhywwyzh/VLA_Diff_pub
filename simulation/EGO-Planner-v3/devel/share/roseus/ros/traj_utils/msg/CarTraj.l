;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::CarTraj)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'CarTraj (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::CARTRAJ")
  (make-package "TRAJ_UTILS::CARTRAJ"))

(in-package "ROS")
;;//! \htmlinclude CarTraj.msg.html


(defclass traj_utils::CarTraj
  :super ros::object
  :slots (_start_stamp _dt _X _id ))

(defmethod traj_utils::CarTraj
  (:init
   (&key
    ((:start_stamp __start_stamp) (instance ros::time :init))
    ((:dt __dt) 0.0)
    ((:X __X) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _start_stamp __start_stamp)
   (setq _dt (float __dt))
   (setq _X __X)
   (setq _id (round __id))
   self)
  (:start_stamp
   (&optional __start_stamp)
   (if __start_stamp (setq _start_stamp __start_stamp)) _start_stamp)
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:X
   (&optional __X)
   (if __X (setq _X __X)) _X)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; time _start_stamp
    8
    ;; float64 _dt
    8
    ;; float64[] _X
    (* 8    (length _X)) 4
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _start_stamp
       (write-long (send _start_stamp :sec) s) (write-long (send _start_stamp :nsec) s)
     ;; float64 _dt
       (sys::poke _dt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _X
     (write-long (length _X) s)
     (dotimes (i (length _X))
       (sys::poke (elt _X i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _start_stamp
     (send _start_stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _dt
     (setq _dt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _X
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _X (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _X i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get traj_utils::CarTraj :md5sum-) "951e389eb9cdbc57941969e38b64973a")
(setf (get traj_utils::CarTraj :datatype-) "traj_utils/CarTraj")
(setf (get traj_utils::CarTraj :definition-)
      "time start_stamp
float64 dt
float64[] X
int32 id

")



(provide :traj_utils/CarTraj "951e389eb9cdbc57941969e38b64973a")


