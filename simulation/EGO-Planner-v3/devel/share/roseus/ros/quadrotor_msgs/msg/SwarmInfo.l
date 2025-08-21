;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::SwarmInfo)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'SwarmInfo (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::SWARMINFO")
  (make-package "QUADROTOR_MSGS::SWARMINFO"))

(in-package "ROS")
;;//! \htmlinclude SwarmInfo.msg.html


(defclass quadrotor_msgs::SwarmInfo
  :super ros::object
  :slots (_swarm_id _robot_ids _leader_id ))

(defmethod quadrotor_msgs::SwarmInfo
  (:init
   (&key
    ((:swarm_id __swarm_id) 0)
    ((:robot_ids __robot_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:leader_id __leader_id) 0)
    )
   (send-super :init)
   (setq _swarm_id (round __swarm_id))
   (setq _robot_ids __robot_ids)
   (setq _leader_id (round __leader_id))
   self)
  (:swarm_id
   (&optional __swarm_id)
   (if __swarm_id (setq _swarm_id __swarm_id)) _swarm_id)
  (:robot_ids
   (&optional __robot_ids)
   (if __robot_ids (setq _robot_ids __robot_ids)) _robot_ids)
  (:leader_id
   (&optional __leader_id)
   (if __leader_id (setq _leader_id __leader_id)) _leader_id)
  (:serialization-length
   ()
   (+
    ;; int32 _swarm_id
    4
    ;; int32[] _robot_ids
    (* 4    (length _robot_ids)) 4
    ;; int32 _leader_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _swarm_id
       (write-long _swarm_id s)
     ;; int32[] _robot_ids
     (write-long (length _robot_ids) s)
     (dotimes (i (length _robot_ids))
       (write-long (elt _robot_ids i) s)
       )
     ;; int32 _leader_id
       (write-long _leader_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _swarm_id
     (setq _swarm_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32[] _robot_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _robot_ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _leader_id
     (setq _leader_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get quadrotor_msgs::SwarmInfo :md5sum-) "7e58f24709b97dba610bf92bfa971d6d")
(setf (get quadrotor_msgs::SwarmInfo :datatype-) "quadrotor_msgs/SwarmInfo")
(setf (get quadrotor_msgs::SwarmInfo :definition-)
      "int32 swarm_id
int32[] robot_ids
int32 leader_id
")



(provide :quadrotor_msgs/SwarmInfo "7e58f24709b97dba610bf92bfa971d6d")


