; Auto-generated. Do not edit!


(cl:in-package quadrotor_msgs-msg)


;//! \htmlinclude PerceptionMsg.msg.html

(cl:defclass <PerceptionMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (from_drone_id
    :reader from_drone_id
    :initarg :from_drone_id
    :type cl:fixnum
    :initform 0)
   (to_drone_id
    :reader to_drone_id
    :initarg :to_drone_id
    :type cl:fixnum
    :initform 0)
   (req_or_resp
    :reader req_or_resp
    :initarg :req_or_resp
    :type cl:fixnum
    :initform 0)
   (msg_type
    :reader msg_type
    :initarg :msg_type
    :type cl:fixnum
    :initform 0)
   (posegraph_msg
    :reader posegraph_msg
    :initarg :posegraph_msg
    :type quadrotor_msgs-msg:MultiPoseGraph
    :initform (cl:make-instance 'quadrotor_msgs-msg:MultiPoseGraph))
   (hgrid_msg
    :reader hgrid_msg
    :initarg :hgrid_msg
    :type quadrotor_msgs-msg:HgridMsg
    :initform (cl:make-instance 'quadrotor_msgs-msg:HgridMsg))
   (ftr_msg
    :reader ftr_msg
    :initarg :ftr_msg
    :type quadrotor_msgs-msg:FrontierMsg
    :initform (cl:make-instance 'quadrotor_msgs-msg:FrontierMsg)))
)

(cl:defclass PerceptionMsg (<PerceptionMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerceptionMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerceptionMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_msgs-msg:<PerceptionMsg> is deprecated: use quadrotor_msgs-msg:PerceptionMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:header-val is deprecated.  Use quadrotor_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'from_drone_id-val :lambda-list '(m))
(cl:defmethod from_drone_id-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:from_drone_id-val is deprecated.  Use quadrotor_msgs-msg:from_drone_id instead.")
  (from_drone_id m))

(cl:ensure-generic-function 'to_drone_id-val :lambda-list '(m))
(cl:defmethod to_drone_id-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:to_drone_id-val is deprecated.  Use quadrotor_msgs-msg:to_drone_id instead.")
  (to_drone_id m))

(cl:ensure-generic-function 'req_or_resp-val :lambda-list '(m))
(cl:defmethod req_or_resp-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:req_or_resp-val is deprecated.  Use quadrotor_msgs-msg:req_or_resp instead.")
  (req_or_resp m))

(cl:ensure-generic-function 'msg_type-val :lambda-list '(m))
(cl:defmethod msg_type-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:msg_type-val is deprecated.  Use quadrotor_msgs-msg:msg_type instead.")
  (msg_type m))

(cl:ensure-generic-function 'posegraph_msg-val :lambda-list '(m))
(cl:defmethod posegraph_msg-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:posegraph_msg-val is deprecated.  Use quadrotor_msgs-msg:posegraph_msg instead.")
  (posegraph_msg m))

(cl:ensure-generic-function 'hgrid_msg-val :lambda-list '(m))
(cl:defmethod hgrid_msg-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:hgrid_msg-val is deprecated.  Use quadrotor_msgs-msg:hgrid_msg instead.")
  (hgrid_msg m))

(cl:ensure-generic-function 'ftr_msg-val :lambda-list '(m))
(cl:defmethod ftr_msg-val ((m <PerceptionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_msgs-msg:ftr_msg-val is deprecated.  Use quadrotor_msgs-msg:ftr_msg instead.")
  (ftr_msg m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<PerceptionMsg>)))
    "Constants for message type '<PerceptionMsg>"
  '((:DATA_NEED_MAP_MERGE . 1)
    (:DATA_NEED_MAP_RESET . 2)
    (:RESPONSE_MSG_FALG . 2)
    (:REQUESET_MSG_FLAG . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'PerceptionMsg)))
    "Constants for message type 'PerceptionMsg"
  '((:DATA_NEED_MAP_MERGE . 1)
    (:DATA_NEED_MAP_RESET . 2)
    (:RESPONSE_MSG_FALG . 2)
    (:REQUESET_MSG_FLAG . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerceptionMsg>) ostream)
  "Serializes a message object of type '<PerceptionMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'from_drone_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'from_drone_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'to_drone_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'to_drone_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'req_or_resp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posegraph_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hgrid_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ftr_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerceptionMsg>) istream)
  "Deserializes a message object of type '<PerceptionMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'from_drone_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'from_drone_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'to_drone_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'to_drone_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'req_or_resp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posegraph_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hgrid_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ftr_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerceptionMsg>)))
  "Returns string type for a message object of type '<PerceptionMsg>"
  "quadrotor_msgs/PerceptionMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerceptionMsg)))
  "Returns string type for a message object of type 'PerceptionMsg"
  "quadrotor_msgs/PerceptionMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerceptionMsg>)))
  "Returns md5sum for a message object of type '<PerceptionMsg>"
  "1dcaf3ff09d27642f7d2a9908eb37ead")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerceptionMsg)))
  "Returns md5sum for a message object of type 'PerceptionMsg"
  "1dcaf3ff09d27642f7d2a9908eb37ead")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerceptionMsg>)))
  "Returns full string definition for message of type '<PerceptionMsg>"
  (cl:format cl:nil "Header header~%# -------------------~%uint16 from_drone_id~%uint16 to_drone_id~%uint8 req_or_resp~%uint8 msg_type~%MultiPoseGraph posegraph_msg~%HgridMsg hgrid_msg~%FrontierMsg ftr_msg~%~%uint8 DATA_NEED_MAP_MERGE = 1~%uint8 DATA_NEED_MAP_RESET = 2~%uint8 RESPONSE_MSG_FALG = 2~%uint8 REQUESET_MSG_FLAG = 1~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/MultiPoseGraph~%Header header~%#  --- only support single pose graph for now --- #~%geometry_msgs/Point[] key_pose_list_xyz~%float32[] key_pose_list_intensity~%~%# p_start no need to tans, because it is bind with keypoint sequences~%std_msgs/UInt16MultiArray[] pose_edge_p_end~%std_msgs/Float32MultiArray[] pose_edge_weight~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/UInt16MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint16[]            data        # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: quadrotor_msgs/HgridMsg~%Header header~%bool recv~%~%# every single grid data (GridInfo)~%uint16[] id~%uint16[] local_id~%uint16[] unknown_num  #~%uint16[] frontier_num # no use~%geometry_msgs/Point[] center~%std_msgs/UInt16MultiArray[] frontier_cell_nums~%std_msgs/UInt16MultiArray[] contained_frontier_ids~%bool[] is_updated~%bool[] need_divide~%bool[] active~%bool[] is_prev_relevant~%bool[] is_cur_relevant~%bool[] is_covered~%~%# multy grid data (UniformGrid)~%std_msgs/UInt16MultiArray relevant_id~%std_msgs/UInt16MultiArray relevant_map~%geometry_msgs/Point min~%geometry_msgs/Point max~%geometry_msgs/Point resolution~%float32 grid_size~%bool multi_layer_hgrid~%~%~%================================================================================~%MSG: quadrotor_msgs/FrontierMsg~%Header header~%# -----------------------~%FtrPointArray[] cells~%FtrPointArray[] filtered_cells~%geometry_msgs/Point[] average~%geometry_msgs/Point[] normal~%uint16[] id~%uint16[] keypose_idx~%# view points for each frontier~%FtrPointArray[] viewpoints_pos~%std_msgs/Float32MultiArray[] viewpoints_yaw~%std_msgs/UInt16MultiArray[] viewpoints_visib_num~%geometry_msgs/Point[] box_min_~%geometry_msgs/Point[] box_max_~%# path & costs between frontiers~%FtrPathArray[] paths~%std_msgs/Float32MultiArray[] costs~%FtrPointArray[] path_to_home_3   # (x, y, z, state)~%std_msgs/Float32MultiArray[] path_to_home_4~%float32[] cost_to_home# (x, y, z, state)~%std_msgs/UInt16MultiArray[] topo_blacklist~%# frontier blacklist (can't reach)~%geometry_msgs/Point[] ftr_blacklist~%================================================================================~%MSG: quadrotor_msgs/FtrPointArray~%geometry_msgs/Point[] PointArray~%================================================================================~%MSG: quadrotor_msgs/FtrPathArray~%FtrPointArray[] PathArray~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerceptionMsg)))
  "Returns full string definition for message of type 'PerceptionMsg"
  (cl:format cl:nil "Header header~%# -------------------~%uint16 from_drone_id~%uint16 to_drone_id~%uint8 req_or_resp~%uint8 msg_type~%MultiPoseGraph posegraph_msg~%HgridMsg hgrid_msg~%FrontierMsg ftr_msg~%~%uint8 DATA_NEED_MAP_MERGE = 1~%uint8 DATA_NEED_MAP_RESET = 2~%uint8 RESPONSE_MSG_FALG = 2~%uint8 REQUESET_MSG_FLAG = 1~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: quadrotor_msgs/MultiPoseGraph~%Header header~%#  --- only support single pose graph for now --- #~%geometry_msgs/Point[] key_pose_list_xyz~%float32[] key_pose_list_intensity~%~%# p_start no need to tans, because it is bind with keypoint sequences~%std_msgs/UInt16MultiArray[] pose_edge_p_end~%std_msgs/Float32MultiArray[] pose_edge_weight~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/UInt16MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%uint16[]            data        # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: quadrotor_msgs/HgridMsg~%Header header~%bool recv~%~%# every single grid data (GridInfo)~%uint16[] id~%uint16[] local_id~%uint16[] unknown_num  #~%uint16[] frontier_num # no use~%geometry_msgs/Point[] center~%std_msgs/UInt16MultiArray[] frontier_cell_nums~%std_msgs/UInt16MultiArray[] contained_frontier_ids~%bool[] is_updated~%bool[] need_divide~%bool[] active~%bool[] is_prev_relevant~%bool[] is_cur_relevant~%bool[] is_covered~%~%# multy grid data (UniformGrid)~%std_msgs/UInt16MultiArray relevant_id~%std_msgs/UInt16MultiArray relevant_map~%geometry_msgs/Point min~%geometry_msgs/Point max~%geometry_msgs/Point resolution~%float32 grid_size~%bool multi_layer_hgrid~%~%~%================================================================================~%MSG: quadrotor_msgs/FrontierMsg~%Header header~%# -----------------------~%FtrPointArray[] cells~%FtrPointArray[] filtered_cells~%geometry_msgs/Point[] average~%geometry_msgs/Point[] normal~%uint16[] id~%uint16[] keypose_idx~%# view points for each frontier~%FtrPointArray[] viewpoints_pos~%std_msgs/Float32MultiArray[] viewpoints_yaw~%std_msgs/UInt16MultiArray[] viewpoints_visib_num~%geometry_msgs/Point[] box_min_~%geometry_msgs/Point[] box_max_~%# path & costs between frontiers~%FtrPathArray[] paths~%std_msgs/Float32MultiArray[] costs~%FtrPointArray[] path_to_home_3   # (x, y, z, state)~%std_msgs/Float32MultiArray[] path_to_home_4~%float32[] cost_to_home# (x, y, z, state)~%std_msgs/UInt16MultiArray[] topo_blacklist~%# frontier blacklist (can't reach)~%geometry_msgs/Point[] ftr_blacklist~%================================================================================~%MSG: quadrotor_msgs/FtrPointArray~%geometry_msgs/Point[] PointArray~%================================================================================~%MSG: quadrotor_msgs/FtrPathArray~%FtrPointArray[] PathArray~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerceptionMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posegraph_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hgrid_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ftr_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerceptionMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PerceptionMsg
    (cl:cons ':header (header msg))
    (cl:cons ':from_drone_id (from_drone_id msg))
    (cl:cons ':to_drone_id (to_drone_id msg))
    (cl:cons ':req_or_resp (req_or_resp msg))
    (cl:cons ':msg_type (msg_type msg))
    (cl:cons ':posegraph_msg (posegraph_msg msg))
    (cl:cons ':hgrid_msg (hgrid_msg msg))
    (cl:cons ':ftr_msg (ftr_msg msg))
))
