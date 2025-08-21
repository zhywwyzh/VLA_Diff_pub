;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::test)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'test (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::TEST")
  (make-package "TRAJ_UTILS::TEST"))

(in-package "ROS")
;;//! \htmlinclude test.msg.html


(defclass traj_utils::test
  :super ros::object
  :slots (_drone_id ))

(defmethod traj_utils::test
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:serialization-length
   ()
   (+
    ;; int8 _drone_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _drone_id
       (write-byte _drone_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _drone_id
     (setq _drone_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _drone_id 127) (setq _drone_id (- _drone_id 256)))
   ;;
   self)
  )

(setf (get traj_utils::test :md5sum-) "6cd4b8fbb9ccb430cf5df0ceccf8d731")
(setf (get traj_utils::test :datatype-) "traj_utils/test")
(setf (get traj_utils::test :definition-)
      "int8 drone_id
")



(provide :traj_utils/test "6cd4b8fbb9ccb430cf5df0ceccf8d731")


