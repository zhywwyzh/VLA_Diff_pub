;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::PerceptionMsg)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'PerceptionMsg (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::PERCEPTIONMSG")
  (make-package "QUADROTOR_MSGS::PERCEPTIONMSG"))

(in-package "ROS")
;;//! \htmlinclude PerceptionMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*DATA_NEED_MAP_MERGE*" (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(shadow '*DATA_NEED_MAP_MERGE* (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(defconstant quadrotor_msgs::PerceptionMsg::*DATA_NEED_MAP_MERGE* 1)
(intern "*DATA_NEED_MAP_RESET*" (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(shadow '*DATA_NEED_MAP_RESET* (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(defconstant quadrotor_msgs::PerceptionMsg::*DATA_NEED_MAP_RESET* 2)
(intern "*RESPONSE_MSG_FALG*" (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(shadow '*RESPONSE_MSG_FALG* (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(defconstant quadrotor_msgs::PerceptionMsg::*RESPONSE_MSG_FALG* 2)
(intern "*REQUESET_MSG_FLAG*" (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(shadow '*REQUESET_MSG_FLAG* (find-package "QUADROTOR_MSGS::PERCEPTIONMSG"))
(defconstant quadrotor_msgs::PerceptionMsg::*REQUESET_MSG_FLAG* 1)

(defun quadrotor_msgs::PerceptionMsg-to-symbol (const)
  (cond
        ((= const 1) 'quadrotor_msgs::PerceptionMsg::*DATA_NEED_MAP_MERGE*)
        ((= const 2) 'quadrotor_msgs::PerceptionMsg::*DATA_NEED_MAP_RESET*)
        ((= const 2) 'quadrotor_msgs::PerceptionMsg::*RESPONSE_MSG_FALG*)
        ((= const 1) 'quadrotor_msgs::PerceptionMsg::*REQUESET_MSG_FLAG*)
        (t nil)))

(defclass quadrotor_msgs::PerceptionMsg
  :super ros::object
  :slots (_header _from_drone_id _to_drone_id _req_or_resp _msg_type _posegraph_msg _hgrid_msg _ftr_msg ))

(defmethod quadrotor_msgs::PerceptionMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:from_drone_id __from_drone_id) 0)
    ((:to_drone_id __to_drone_id) 0)
    ((:req_or_resp __req_or_resp) 0)
    ((:msg_type __msg_type) 0)
    ((:posegraph_msg __posegraph_msg) (instance quadrotor_msgs::MultiPoseGraph :init))
    ((:hgrid_msg __hgrid_msg) (instance quadrotor_msgs::HgridMsg :init))
    ((:ftr_msg __ftr_msg) (instance quadrotor_msgs::FrontierMsg :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _from_drone_id (round __from_drone_id))
   (setq _to_drone_id (round __to_drone_id))
   (setq _req_or_resp (round __req_or_resp))
   (setq _msg_type (round __msg_type))
   (setq _posegraph_msg __posegraph_msg)
   (setq _hgrid_msg __hgrid_msg)
   (setq _ftr_msg __ftr_msg)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:from_drone_id
   (&optional __from_drone_id)
   (if __from_drone_id (setq _from_drone_id __from_drone_id)) _from_drone_id)
  (:to_drone_id
   (&optional __to_drone_id)
   (if __to_drone_id (setq _to_drone_id __to_drone_id)) _to_drone_id)
  (:req_or_resp
   (&optional __req_or_resp)
   (if __req_or_resp (setq _req_or_resp __req_or_resp)) _req_or_resp)
  (:msg_type
   (&optional __msg_type)
   (if __msg_type (setq _msg_type __msg_type)) _msg_type)
  (:posegraph_msg
   (&rest __posegraph_msg)
   (if (keywordp (car __posegraph_msg))
       (send* _posegraph_msg __posegraph_msg)
     (progn
       (if __posegraph_msg (setq _posegraph_msg (car __posegraph_msg)))
       _posegraph_msg)))
  (:hgrid_msg
   (&rest __hgrid_msg)
   (if (keywordp (car __hgrid_msg))
       (send* _hgrid_msg __hgrid_msg)
     (progn
       (if __hgrid_msg (setq _hgrid_msg (car __hgrid_msg)))
       _hgrid_msg)))
  (:ftr_msg
   (&rest __ftr_msg)
   (if (keywordp (car __ftr_msg))
       (send* _ftr_msg __ftr_msg)
     (progn
       (if __ftr_msg (setq _ftr_msg (car __ftr_msg)))
       _ftr_msg)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _from_drone_id
    2
    ;; uint16 _to_drone_id
    2
    ;; uint8 _req_or_resp
    1
    ;; uint8 _msg_type
    1
    ;; quadrotor_msgs/MultiPoseGraph _posegraph_msg
    (send _posegraph_msg :serialization-length)
    ;; quadrotor_msgs/HgridMsg _hgrid_msg
    (send _hgrid_msg :serialization-length)
    ;; quadrotor_msgs/FrontierMsg _ftr_msg
    (send _ftr_msg :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _from_drone_id
       (write-word _from_drone_id s)
     ;; uint16 _to_drone_id
       (write-word _to_drone_id s)
     ;; uint8 _req_or_resp
       (write-byte _req_or_resp s)
     ;; uint8 _msg_type
       (write-byte _msg_type s)
     ;; quadrotor_msgs/MultiPoseGraph _posegraph_msg
       (send _posegraph_msg :serialize s)
     ;; quadrotor_msgs/HgridMsg _hgrid_msg
       (send _hgrid_msg :serialize s)
     ;; quadrotor_msgs/FrontierMsg _ftr_msg
       (send _ftr_msg :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _from_drone_id
     (setq _from_drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _to_drone_id
     (setq _to_drone_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _req_or_resp
     (setq _req_or_resp (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _msg_type
     (setq _msg_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; quadrotor_msgs/MultiPoseGraph _posegraph_msg
     (send _posegraph_msg :deserialize buf ptr-) (incf ptr- (send _posegraph_msg :serialization-length))
   ;; quadrotor_msgs/HgridMsg _hgrid_msg
     (send _hgrid_msg :deserialize buf ptr-) (incf ptr- (send _hgrid_msg :serialization-length))
   ;; quadrotor_msgs/FrontierMsg _ftr_msg
     (send _ftr_msg :deserialize buf ptr-) (incf ptr- (send _ftr_msg :serialization-length))
   ;;
   self)
  )

(setf (get quadrotor_msgs::PerceptionMsg :md5sum-) "1dcaf3ff09d27642f7d2a9908eb37ead")
(setf (get quadrotor_msgs::PerceptionMsg :datatype-) "quadrotor_msgs/PerceptionMsg")
(setf (get quadrotor_msgs::PerceptionMsg :definition-)
      "Header header
# -------------------
uint16 from_drone_id
uint16 to_drone_id
uint8 req_or_resp
uint8 msg_type
MultiPoseGraph posegraph_msg
HgridMsg hgrid_msg
FrontierMsg ftr_msg

uint8 DATA_NEED_MAP_MERGE = 1
uint8 DATA_NEED_MAP_RESET = 2
uint8 RESPONSE_MSG_FALG = 2
uint8 REQUESET_MSG_FLAG = 1
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
MSG: quadrotor_msgs/MultiPoseGraph
Header header
#  --- only support single pose graph for now --- #
geometry_msgs/Point[] key_pose_list_xyz
float32[] key_pose_list_intensity

# p_start no need to tans, because it is bind with keypoint sequences
std_msgs/UInt16MultiArray[] pose_edge_p_end
std_msgs/Float32MultiArray[] pose_edge_weight

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


================================================================================
MSG: quadrotor_msgs/HgridMsg
Header header
bool recv

# every single grid data (GridInfo)
uint16[] id
uint16[] local_id
uint16[] unknown_num  #
uint16[] frontier_num # no use
geometry_msgs/Point[] center
std_msgs/UInt16MultiArray[] frontier_cell_nums
std_msgs/UInt16MultiArray[] contained_frontier_ids
bool[] is_updated
bool[] need_divide
bool[] active
bool[] is_prev_relevant
bool[] is_cur_relevant
bool[] is_covered

# multy grid data (UniformGrid)
std_msgs/UInt16MultiArray relevant_id
std_msgs/UInt16MultiArray relevant_map
geometry_msgs/Point min
geometry_msgs/Point max
geometry_msgs/Point resolution
float32 grid_size
bool multi_layer_hgrid


================================================================================
MSG: quadrotor_msgs/FrontierMsg
Header header
# -----------------------
FtrPointArray[] cells
FtrPointArray[] filtered_cells
geometry_msgs/Point[] average
geometry_msgs/Point[] normal
uint16[] id
uint16[] keypose_idx
# view points for each frontier
FtrPointArray[] viewpoints_pos
std_msgs/Float32MultiArray[] viewpoints_yaw
std_msgs/UInt16MultiArray[] viewpoints_visib_num
geometry_msgs/Point[] box_min_
geometry_msgs/Point[] box_max_
# path & costs between frontiers
FtrPathArray[] paths
std_msgs/Float32MultiArray[] costs
FtrPointArray[] path_to_home_3   # (x, y, z, state)
std_msgs/Float32MultiArray[] path_to_home_4
float32[] cost_to_home# (x, y, z, state)
std_msgs/UInt16MultiArray[] topo_blacklist
# frontier blacklist (can't reach)
geometry_msgs/Point[] ftr_blacklist
================================================================================
MSG: quadrotor_msgs/FtrPointArray
geometry_msgs/Point[] PointArray
================================================================================
MSG: quadrotor_msgs/FtrPathArray
FtrPointArray[] PathArray

")



(provide :quadrotor_msgs/PerceptionMsg "1dcaf3ff09d27642f7d2a9908eb37ead")


