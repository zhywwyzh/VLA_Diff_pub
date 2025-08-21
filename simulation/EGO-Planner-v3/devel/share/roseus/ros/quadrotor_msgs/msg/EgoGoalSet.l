;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::EgoGoalSet)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'EgoGoalSet (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::EGOGOALSET")
  (make-package "QUADROTOR_MSGS::EGOGOALSET"))

(in-package "ROS")
;;//! \htmlinclude EgoGoalSet.msg.html


(defclass quadrotor_msgs::EgoGoalSet
  :super ros::object
  :slots (_drone_id _goal _yaw _look_forward _goal_to_follower ))

(defmethod quadrotor_msgs::EgoGoalSet
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:goal __goal) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:yaw __yaw) 0.0)
    ((:look_forward __look_forward) nil)
    ((:goal_to_follower __goal_to_follower) nil)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _goal __goal)
   (setq _yaw (float __yaw))
   (setq _look_forward __look_forward)
   (setq _goal_to_follower __goal_to_follower)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:look_forward
   (&optional (__look_forward :null))
   (if (not (eq __look_forward :null)) (setq _look_forward __look_forward)) _look_forward)
  (:goal_to_follower
   (&optional (__goal_to_follower :null))
   (if (not (eq __goal_to_follower :null)) (setq _goal_to_follower __goal_to_follower)) _goal_to_follower)
  (:serialization-length
   ()
   (+
    ;; uint8 _drone_id
    1
    ;; float32[3] _goal
    (* 4    3)
    ;; float32 _yaw
    4
    ;; bool _look_forward
    1
    ;; bool _goal_to_follower
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _drone_id
       (write-byte _drone_id s)
     ;; float32[3] _goal
     (dotimes (i 3)
       (sys::poke (elt _goal i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _look_forward
       (if _look_forward (write-byte -1 s) (write-byte 0 s))
     ;; bool _goal_to_follower
       (if _goal_to_follower (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _drone_id
     (setq _drone_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32[3] _goal
   (dotimes (i (length _goal))
     (setf (elt _goal i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _look_forward
     (setq _look_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _goal_to_follower
     (setq _goal_to_follower (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get quadrotor_msgs::EgoGoalSet :md5sum-) "4e000e06493c05ae8165574a33ffc993")
(setf (get quadrotor_msgs::EgoGoalSet :datatype-) "quadrotor_msgs/EgoGoalSet")
(setf (get quadrotor_msgs::EgoGoalSet :definition-)
      "uint8      drone_id
float32[3] goal
float32    yaw
bool       look_forward
bool       goal_to_follower
")



(provide :quadrotor_msgs/EgoGoalSet "4e000e06493c05ae8165574a33ffc993")


