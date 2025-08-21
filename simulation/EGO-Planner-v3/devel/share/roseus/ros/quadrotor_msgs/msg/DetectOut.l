;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::DetectOut)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'DetectOut (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::DETECTOUT")
  (make-package "QUADROTOR_MSGS::DETECTOUT"))

(in-package "ROS")
;;//! \htmlinclude DetectOut.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::DetectOut
  :super ros::object
  :slots (_header _image _image_camera _classes _global_poses ))

(defmethod quadrotor_msgs::DetectOut
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:image_camera __image_camera) "")
    ((:classes __classes) (make-array 0 :initial-element 0 :element-type :integer))
    ((:global_poses __global_poses) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _image __image)
   (setq _image_camera (string __image_camera))
   (setq _classes __classes)
   (setq _global_poses __global_poses)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:image_camera
   (&optional __image_camera)
   (if __image_camera (setq _image_camera __image_camera)) _image_camera)
  (:classes
   (&optional __classes)
   (if __classes (setq _classes __classes)) _classes)
  (:global_poses
   (&rest __global_poses)
   (if (keywordp (car __global_poses))
       (send* _global_poses __global_poses)
     (progn
       (if __global_poses (setq _global_poses (car __global_poses)))
       _global_poses)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; string _image_camera
    4 (length _image_camera)
    ;; int32[] _classes
    (* 4    (length _classes)) 4
    ;; geometry_msgs/Point[] _global_poses
    (apply #'+ (send-all _global_poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; string _image_camera
       (write-long (length _image_camera) s) (princ _image_camera s)
     ;; int32[] _classes
     (write-long (length _classes) s)
     (dotimes (i (length _classes))
       (write-long (elt _classes i) s)
       )
     ;; geometry_msgs/Point[] _global_poses
     (write-long (length _global_poses) s)
     (dolist (elem _global_poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; string _image_camera
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _image_camera (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32[] _classes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _classes (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _classes i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; geometry_msgs/Point[] _global_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _global_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _global_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get quadrotor_msgs::DetectOut :md5sum-) "3a4bd95d10ac107281ee8b0f00b03c02")
(setf (get quadrotor_msgs::DetectOut :datatype-) "quadrotor_msgs/DetectOut")
(setf (get quadrotor_msgs::DetectOut :definition-)
      "std_msgs/Header header
sensor_msgs/Image image
string image_camera
int32[] classes
geometry_msgs/Point[] global_poses
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :quadrotor_msgs/DetectOut "3a4bd95d10ac107281ee8b0f00b03c02")


