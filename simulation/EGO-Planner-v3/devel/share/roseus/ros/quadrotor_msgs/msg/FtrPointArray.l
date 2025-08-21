;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::FtrPointArray)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'FtrPointArray (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::FTRPOINTARRAY")
  (make-package "QUADROTOR_MSGS::FTRPOINTARRAY"))

(in-package "ROS")
;;//! \htmlinclude FtrPointArray.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass quadrotor_msgs::FtrPointArray
  :super ros::object
  :slots (_PointArray ))

(defmethod quadrotor_msgs::FtrPointArray
  (:init
   (&key
    ((:PointArray __PointArray) ())
    )
   (send-super :init)
   (setq _PointArray __PointArray)
   self)
  (:PointArray
   (&rest __PointArray)
   (if (keywordp (car __PointArray))
       (send* _PointArray __PointArray)
     (progn
       (if __PointArray (setq _PointArray (car __PointArray)))
       _PointArray)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _PointArray
    (apply #'+ (send-all _PointArray :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _PointArray
     (write-long (length _PointArray) s)
     (dolist (elem _PointArray)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _PointArray
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _PointArray (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _PointArray)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get quadrotor_msgs::FtrPointArray :md5sum-) "b72da7cba93373ecc2fd54ff85f47989")
(setf (get quadrotor_msgs::FtrPointArray :datatype-) "quadrotor_msgs/FtrPointArray")
(setf (get quadrotor_msgs::FtrPointArray :definition-)
      "geometry_msgs/Point[] PointArray
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :quadrotor_msgs/FtrPointArray "b72da7cba93373ecc2fd54ff85f47989")


