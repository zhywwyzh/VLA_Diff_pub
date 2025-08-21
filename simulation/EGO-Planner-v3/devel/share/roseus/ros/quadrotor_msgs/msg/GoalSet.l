;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::GoalSet)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'GoalSet (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::GOALSET")
  (make-package "QUADROTOR_MSGS::GOALSET"))

(in-package "ROS")
;;//! \htmlinclude GoalSet.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quadrotor_msgs::GoalSet
  :super ros::object
  :slots (_to_drone_ids _goal _yaw _look_forward _goal_to_follower ))

(defmethod quadrotor_msgs::GoalSet
  (:init
   (&key
    ((:to_drone_ids __to_drone_ids) (make-array 0 :initial-element 0 :element-type :char))
    ((:goal __goal) ())
    ((:yaw __yaw) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:look_forward __look_forward) nil)
    ((:goal_to_follower __goal_to_follower) nil)
    )
   (send-super :init)
   (setq _to_drone_ids __to_drone_ids)
   (setq _goal __goal)
   (setq _yaw __yaw)
   (setq _look_forward __look_forward)
   (setq _goal_to_follower __goal_to_follower)
   self)
  (:to_drone_ids
   (&optional __to_drone_ids)
   (if __to_drone_ids (setq _to_drone_ids __to_drone_ids)) _to_drone_ids)
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
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
    ;; uint8[] _to_drone_ids
    (* 1    (length _to_drone_ids)) 4
    ;; geometry_msgs/Point[] _goal
    (apply #'+ (send-all _goal :serialization-length)) 4
    ;; float32[] _yaw
    (* 4    (length _yaw)) 4
    ;; bool _look_forward
    1
    ;; bool _goal_to_follower
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _to_drone_ids
     (write-long (length _to_drone_ids) s)
     (princ _to_drone_ids s)
     ;; geometry_msgs/Point[] _goal
     (write-long (length _goal) s)
     (dolist (elem _goal)
       (send elem :serialize s)
       )
     ;; float32[] _yaw
     (write-long (length _yaw) s)
     (dotimes (i (length _yaw))
       (sys::poke (elt _yaw i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _look_forward
       (if _look_forward (write-byte -1 s) (write-byte 0 s))
     ;; bool _goal_to_follower
       (if _goal_to_follower (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _to_drone_ids
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _to_drone_ids (make-array n :element-type :char))
     (replace _to_drone_ids buf :start2 ptr-) (incf ptr- n))
   ;; geometry_msgs/Point[] _goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _goal)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _yaw
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yaw (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _yaw i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _look_forward
     (setq _look_forward (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _goal_to_follower
     (setq _goal_to_follower (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get quadrotor_msgs::GoalSet :md5sum-) "fce849fd3f9e593aecb0eb34b6f685a3")
(setf (get quadrotor_msgs::GoalSet :datatype-) "quadrotor_msgs/GoalSet")
(setf (get quadrotor_msgs::GoalSet :definition-)
      "uint8[]               to_drone_ids
geometry_msgs/Point[] goal
float32[]             yaw
bool                  look_forward
bool                  goal_to_follower
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :quadrotor_msgs/GoalSet "fce849fd3f9e593aecb0eb34b6f685a3")


