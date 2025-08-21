;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::MultiPoseGraph)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'MultiPoseGraph (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::MULTIPOSEGRAPH")
  (make-package "QUADROTOR_MSGS::MULTIPOSEGRAPH"))

(in-package "ROS")
;;//! \htmlinclude MultiPoseGraph.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::MultiPoseGraph
  :super ros::object
  :slots (_header _key_pose_list_xyz _key_pose_list_intensity _pose_edge_p_end _pose_edge_weight ))

(defmethod quadrotor_msgs::MultiPoseGraph
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:key_pose_list_xyz __key_pose_list_xyz) ())
    ((:key_pose_list_intensity __key_pose_list_intensity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pose_edge_p_end __pose_edge_p_end) ())
    ((:pose_edge_weight __pose_edge_weight) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _key_pose_list_xyz __key_pose_list_xyz)
   (setq _key_pose_list_intensity __key_pose_list_intensity)
   (setq _pose_edge_p_end __pose_edge_p_end)
   (setq _pose_edge_weight __pose_edge_weight)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:key_pose_list_xyz
   (&rest __key_pose_list_xyz)
   (if (keywordp (car __key_pose_list_xyz))
       (send* _key_pose_list_xyz __key_pose_list_xyz)
     (progn
       (if __key_pose_list_xyz (setq _key_pose_list_xyz (car __key_pose_list_xyz)))
       _key_pose_list_xyz)))
  (:key_pose_list_intensity
   (&optional __key_pose_list_intensity)
   (if __key_pose_list_intensity (setq _key_pose_list_intensity __key_pose_list_intensity)) _key_pose_list_intensity)
  (:pose_edge_p_end
   (&rest __pose_edge_p_end)
   (if (keywordp (car __pose_edge_p_end))
       (send* _pose_edge_p_end __pose_edge_p_end)
     (progn
       (if __pose_edge_p_end (setq _pose_edge_p_end (car __pose_edge_p_end)))
       _pose_edge_p_end)))
  (:pose_edge_weight
   (&rest __pose_edge_weight)
   (if (keywordp (car __pose_edge_weight))
       (send* _pose_edge_weight __pose_edge_weight)
     (progn
       (if __pose_edge_weight (setq _pose_edge_weight (car __pose_edge_weight)))
       _pose_edge_weight)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _key_pose_list_xyz
    (apply #'+ (send-all _key_pose_list_xyz :serialization-length)) 4
    ;; float32[] _key_pose_list_intensity
    (* 4    (length _key_pose_list_intensity)) 4
    ;; std_msgs/UInt16MultiArray[] _pose_edge_p_end
    (apply #'+ (send-all _pose_edge_p_end :serialization-length)) 4
    ;; std_msgs/Float32MultiArray[] _pose_edge_weight
    (apply #'+ (send-all _pose_edge_weight :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _key_pose_list_xyz
     (write-long (length _key_pose_list_xyz) s)
     (dolist (elem _key_pose_list_xyz)
       (send elem :serialize s)
       )
     ;; float32[] _key_pose_list_intensity
     (write-long (length _key_pose_list_intensity) s)
     (dotimes (i (length _key_pose_list_intensity))
       (sys::poke (elt _key_pose_list_intensity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; std_msgs/UInt16MultiArray[] _pose_edge_p_end
     (write-long (length _pose_edge_p_end) s)
     (dolist (elem _pose_edge_p_end)
       (send elem :serialize s)
       )
     ;; std_msgs/Float32MultiArray[] _pose_edge_weight
     (write-long (length _pose_edge_weight) s)
     (dolist (elem _pose_edge_weight)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _key_pose_list_xyz
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _key_pose_list_xyz (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _key_pose_list_xyz)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _key_pose_list_intensity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _key_pose_list_intensity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _key_pose_list_intensity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; std_msgs/UInt16MultiArray[] _pose_edge_p_end
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose_edge_p_end (let (r) (dotimes (i n) (push (instance std_msgs::UInt16MultiArray :init) r)) r))
     (dolist (elem- _pose_edge_p_end)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float32MultiArray[] _pose_edge_weight
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose_edge_weight (let (r) (dotimes (i n) (push (instance std_msgs::Float32MultiArray :init) r)) r))
     (dolist (elem- _pose_edge_weight)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get quadrotor_msgs::MultiPoseGraph :md5sum-) "1253e98cd3689b7af6c77b6941e0ebdc")
(setf (get quadrotor_msgs::MultiPoseGraph :datatype-) "quadrotor_msgs/MultiPoseGraph")
(setf (get quadrotor_msgs::MultiPoseGraph :definition-)
      "Header header
#  --- only support single pose graph for now --- #
geometry_msgs/Point[] key_pose_list_xyz
float32[] key_pose_list_intensity

# p_start no need to tans, because it is bind with keypoint sequences
std_msgs/UInt16MultiArray[] pose_edge_p_end
std_msgs/Float32MultiArray[] pose_edge_weight

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/UInt16MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint16[]            data        # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


")



(provide :quadrotor_msgs/MultiPoseGraph "1253e98cd3689b7af6c77b6941e0ebdc")


