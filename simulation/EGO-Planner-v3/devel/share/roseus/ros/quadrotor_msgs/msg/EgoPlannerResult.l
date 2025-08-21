;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::EgoPlannerResult)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'EgoPlannerResult (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::EGOPLANNERRESULT")
  (make-package "QUADROTOR_MSGS::EGOPLANNERRESULT"))

(in-package "ROS")
;;//! \htmlinclude EgoPlannerResult.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quadrotor_msgs::EgoPlannerResult
  :super ros::object
  :slots (_planner_goal _plan_times _plan_status _modify_status ))

(defmethod quadrotor_msgs::EgoPlannerResult
  (:init
   (&key
    ((:planner_goal __planner_goal) (instance geometry_msgs::Vector3 :init))
    ((:plan_times __plan_times) 0)
    ((:plan_status __plan_status) nil)
    ((:modify_status __modify_status) nil)
    )
   (send-super :init)
   (setq _planner_goal __planner_goal)
   (setq _plan_times (round __plan_times))
   (setq _plan_status __plan_status)
   (setq _modify_status __modify_status)
   self)
  (:planner_goal
   (&rest __planner_goal)
   (if (keywordp (car __planner_goal))
       (send* _planner_goal __planner_goal)
     (progn
       (if __planner_goal (setq _planner_goal (car __planner_goal)))
       _planner_goal)))
  (:plan_times
   (&optional __plan_times)
   (if __plan_times (setq _plan_times __plan_times)) _plan_times)
  (:plan_status
   (&optional (__plan_status :null))
   (if (not (eq __plan_status :null)) (setq _plan_status __plan_status)) _plan_status)
  (:modify_status
   (&optional (__modify_status :null))
   (if (not (eq __modify_status :null)) (setq _modify_status __modify_status)) _modify_status)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _planner_goal
    (send _planner_goal :serialization-length)
    ;; int16 _plan_times
    2
    ;; bool _plan_status
    1
    ;; bool _modify_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _planner_goal
       (send _planner_goal :serialize s)
     ;; int16 _plan_times
       (write-word _plan_times s)
     ;; bool _plan_status
       (if _plan_status (write-byte -1 s) (write-byte 0 s))
     ;; bool _modify_status
       (if _modify_status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _planner_goal
     (send _planner_goal :deserialize buf ptr-) (incf ptr- (send _planner_goal :serialization-length))
   ;; int16 _plan_times
     (setq _plan_times (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _plan_status
     (setq _plan_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _modify_status
     (setq _modify_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get quadrotor_msgs::EgoPlannerResult :md5sum-) "e6cf40d72e8bfa9282deb6b127bfe25d")
(setf (get quadrotor_msgs::EgoPlannerResult :datatype-) "quadrotor_msgs/EgoPlannerResult")
(setf (get quadrotor_msgs::EgoPlannerResult :definition-)
      "#data structure
geometry_msgs/Vector3 planner_goal
int16 plan_times
bool plan_status
bool modify_status

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :quadrotor_msgs/EgoPlannerResult "e6cf40d72e8bfa9282deb6b127bfe25d")


