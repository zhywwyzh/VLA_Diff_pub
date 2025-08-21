;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::FtrPathArray)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'FtrPathArray (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::FTRPATHARRAY")
  (make-package "QUADROTOR_MSGS::FTRPATHARRAY"))

(in-package "ROS")
;;//! \htmlinclude FtrPathArray.msg.html


(defclass quadrotor_msgs::FtrPathArray
  :super ros::object
  :slots (_PathArray ))

(defmethod quadrotor_msgs::FtrPathArray
  (:init
   (&key
    ((:PathArray __PathArray) ())
    )
   (send-super :init)
   (setq _PathArray __PathArray)
   self)
  (:PathArray
   (&rest __PathArray)
   (if (keywordp (car __PathArray))
       (send* _PathArray __PathArray)
     (progn
       (if __PathArray (setq _PathArray (car __PathArray)))
       _PathArray)))
  (:serialization-length
   ()
   (+
    ;; quadrotor_msgs/FtrPointArray[] _PathArray
    (apply #'+ (send-all _PathArray :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; quadrotor_msgs/FtrPointArray[] _PathArray
     (write-long (length _PathArray) s)
     (dolist (elem _PathArray)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; quadrotor_msgs/FtrPointArray[] _PathArray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _PathArray (let (r) (dotimes (i n) (push (instance quadrotor_msgs::FtrPointArray :init) r)) r))
     (dolist (elem- _PathArray)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get quadrotor_msgs::FtrPathArray :md5sum-) "d168064edf658ded54eac8a732d98b8f")
(setf (get quadrotor_msgs::FtrPathArray :datatype-) "quadrotor_msgs/FtrPathArray")
(setf (get quadrotor_msgs::FtrPathArray :definition-)
      "FtrPointArray[] PathArray

================================================================================
MSG: quadrotor_msgs/FtrPointArray
geometry_msgs/Point[] PointArray
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :quadrotor_msgs/FtrPathArray "d168064edf658ded54eac8a732d98b8f")


