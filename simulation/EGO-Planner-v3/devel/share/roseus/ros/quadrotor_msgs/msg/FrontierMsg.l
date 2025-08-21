;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::FrontierMsg)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'FrontierMsg (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::FRONTIERMSG")
  (make-package "QUADROTOR_MSGS::FRONTIERMSG"))

(in-package "ROS")
;;//! \htmlinclude FrontierMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::FrontierMsg
  :super ros::object
  :slots (_header _cells _filtered_cells _average _normal _id _keypose_idx _viewpoints_pos _viewpoints_yaw _viewpoints_visib_num _box_min_ _box_max_ _paths _costs _path_to_home_3 _path_to_home_4 _cost_to_home _topo_blacklist _ftr_blacklist ))

(defmethod quadrotor_msgs::FrontierMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cells __cells) ())
    ((:filtered_cells __filtered_cells) ())
    ((:average __average) ())
    ((:normal __normal) ())
    ((:id __id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:keypose_idx __keypose_idx) (make-array 0 :initial-element 0 :element-type :integer))
    ((:viewpoints_pos __viewpoints_pos) ())
    ((:viewpoints_yaw __viewpoints_yaw) ())
    ((:viewpoints_visib_num __viewpoints_visib_num) ())
    ((:box_min_ __box_min_) ())
    ((:box_max_ __box_max_) ())
    ((:paths __paths) ())
    ((:costs __costs) ())
    ((:path_to_home_3 __path_to_home_3) ())
    ((:path_to_home_4 __path_to_home_4) ())
    ((:cost_to_home __cost_to_home) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:topo_blacklist __topo_blacklist) ())
    ((:ftr_blacklist __ftr_blacklist) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _cells __cells)
   (setq _filtered_cells __filtered_cells)
   (setq _average __average)
   (setq _normal __normal)
   (setq _id __id)
   (setq _keypose_idx __keypose_idx)
   (setq _viewpoints_pos __viewpoints_pos)
   (setq _viewpoints_yaw __viewpoints_yaw)
   (setq _viewpoints_visib_num __viewpoints_visib_num)
   (setq _box_min_ __box_min_)
   (setq _box_max_ __box_max_)
   (setq _paths __paths)
   (setq _costs __costs)
   (setq _path_to_home_3 __path_to_home_3)
   (setq _path_to_home_4 __path_to_home_4)
   (setq _cost_to_home __cost_to_home)
   (setq _topo_blacklist __topo_blacklist)
   (setq _ftr_blacklist __ftr_blacklist)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cells
   (&rest __cells)
   (if (keywordp (car __cells))
       (send* _cells __cells)
     (progn
       (if __cells (setq _cells (car __cells)))
       _cells)))
  (:filtered_cells
   (&rest __filtered_cells)
   (if (keywordp (car __filtered_cells))
       (send* _filtered_cells __filtered_cells)
     (progn
       (if __filtered_cells (setq _filtered_cells (car __filtered_cells)))
       _filtered_cells)))
  (:average
   (&rest __average)
   (if (keywordp (car __average))
       (send* _average __average)
     (progn
       (if __average (setq _average (car __average)))
       _average)))
  (:normal
   (&rest __normal)
   (if (keywordp (car __normal))
       (send* _normal __normal)
     (progn
       (if __normal (setq _normal (car __normal)))
       _normal)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:keypose_idx
   (&optional __keypose_idx)
   (if __keypose_idx (setq _keypose_idx __keypose_idx)) _keypose_idx)
  (:viewpoints_pos
   (&rest __viewpoints_pos)
   (if (keywordp (car __viewpoints_pos))
       (send* _viewpoints_pos __viewpoints_pos)
     (progn
       (if __viewpoints_pos (setq _viewpoints_pos (car __viewpoints_pos)))
       _viewpoints_pos)))
  (:viewpoints_yaw
   (&rest __viewpoints_yaw)
   (if (keywordp (car __viewpoints_yaw))
       (send* _viewpoints_yaw __viewpoints_yaw)
     (progn
       (if __viewpoints_yaw (setq _viewpoints_yaw (car __viewpoints_yaw)))
       _viewpoints_yaw)))
  (:viewpoints_visib_num
   (&rest __viewpoints_visib_num)
   (if (keywordp (car __viewpoints_visib_num))
       (send* _viewpoints_visib_num __viewpoints_visib_num)
     (progn
       (if __viewpoints_visib_num (setq _viewpoints_visib_num (car __viewpoints_visib_num)))
       _viewpoints_visib_num)))
  (:box_min_
   (&rest __box_min_)
   (if (keywordp (car __box_min_))
       (send* _box_min_ __box_min_)
     (progn
       (if __box_min_ (setq _box_min_ (car __box_min_)))
       _box_min_)))
  (:box_max_
   (&rest __box_max_)
   (if (keywordp (car __box_max_))
       (send* _box_max_ __box_max_)
     (progn
       (if __box_max_ (setq _box_max_ (car __box_max_)))
       _box_max_)))
  (:paths
   (&rest __paths)
   (if (keywordp (car __paths))
       (send* _paths __paths)
     (progn
       (if __paths (setq _paths (car __paths)))
       _paths)))
  (:costs
   (&rest __costs)
   (if (keywordp (car __costs))
       (send* _costs __costs)
     (progn
       (if __costs (setq _costs (car __costs)))
       _costs)))
  (:path_to_home_3
   (&rest __path_to_home_3)
   (if (keywordp (car __path_to_home_3))
       (send* _path_to_home_3 __path_to_home_3)
     (progn
       (if __path_to_home_3 (setq _path_to_home_3 (car __path_to_home_3)))
       _path_to_home_3)))
  (:path_to_home_4
   (&rest __path_to_home_4)
   (if (keywordp (car __path_to_home_4))
       (send* _path_to_home_4 __path_to_home_4)
     (progn
       (if __path_to_home_4 (setq _path_to_home_4 (car __path_to_home_4)))
       _path_to_home_4)))
  (:cost_to_home
   (&optional __cost_to_home)
   (if __cost_to_home (setq _cost_to_home __cost_to_home)) _cost_to_home)
  (:topo_blacklist
   (&rest __topo_blacklist)
   (if (keywordp (car __topo_blacklist))
       (send* _topo_blacklist __topo_blacklist)
     (progn
       (if __topo_blacklist (setq _topo_blacklist (car __topo_blacklist)))
       _topo_blacklist)))
  (:ftr_blacklist
   (&rest __ftr_blacklist)
   (if (keywordp (car __ftr_blacklist))
       (send* _ftr_blacklist __ftr_blacklist)
     (progn
       (if __ftr_blacklist (setq _ftr_blacklist (car __ftr_blacklist)))
       _ftr_blacklist)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; quadrotor_msgs/FtrPointArray[] _cells
    (apply #'+ (send-all _cells :serialization-length)) 4
    ;; quadrotor_msgs/FtrPointArray[] _filtered_cells
    (apply #'+ (send-all _filtered_cells :serialization-length)) 4
    ;; geometry_msgs/Point[] _average
    (apply #'+ (send-all _average :serialization-length)) 4
    ;; geometry_msgs/Point[] _normal
    (apply #'+ (send-all _normal :serialization-length)) 4
    ;; uint16[] _id
    (* 2    (length _id)) 4
    ;; uint16[] _keypose_idx
    (* 2    (length _keypose_idx)) 4
    ;; quadrotor_msgs/FtrPointArray[] _viewpoints_pos
    (apply #'+ (send-all _viewpoints_pos :serialization-length)) 4
    ;; std_msgs/Float32MultiArray[] _viewpoints_yaw
    (apply #'+ (send-all _viewpoints_yaw :serialization-length)) 4
    ;; std_msgs/UInt16MultiArray[] _viewpoints_visib_num
    (apply #'+ (send-all _viewpoints_visib_num :serialization-length)) 4
    ;; geometry_msgs/Point[] _box_min_
    (apply #'+ (send-all _box_min_ :serialization-length)) 4
    ;; geometry_msgs/Point[] _box_max_
    (apply #'+ (send-all _box_max_ :serialization-length)) 4
    ;; quadrotor_msgs/FtrPathArray[] _paths
    (apply #'+ (send-all _paths :serialization-length)) 4
    ;; std_msgs/Float32MultiArray[] _costs
    (apply #'+ (send-all _costs :serialization-length)) 4
    ;; quadrotor_msgs/FtrPointArray[] _path_to_home_3
    (apply #'+ (send-all _path_to_home_3 :serialization-length)) 4
    ;; std_msgs/Float32MultiArray[] _path_to_home_4
    (apply #'+ (send-all _path_to_home_4 :serialization-length)) 4
    ;; float32[] _cost_to_home
    (* 4    (length _cost_to_home)) 4
    ;; std_msgs/UInt16MultiArray[] _topo_blacklist
    (apply #'+ (send-all _topo_blacklist :serialization-length)) 4
    ;; geometry_msgs/Point[] _ftr_blacklist
    (apply #'+ (send-all _ftr_blacklist :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; quadrotor_msgs/FtrPointArray[] _cells
     (write-long (length _cells) s)
     (dolist (elem _cells)
       (send elem :serialize s)
       )
     ;; quadrotor_msgs/FtrPointArray[] _filtered_cells
     (write-long (length _filtered_cells) s)
     (dolist (elem _filtered_cells)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _average
     (write-long (length _average) s)
     (dolist (elem _average)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _normal
     (write-long (length _normal) s)
     (dolist (elem _normal)
       (send elem :serialize s)
       )
     ;; uint16[] _id
     (write-long (length _id) s)
     (dotimes (i (length _id))
       (write-word (elt _id i) s)
       )
     ;; uint16[] _keypose_idx
     (write-long (length _keypose_idx) s)
     (dotimes (i (length _keypose_idx))
       (write-word (elt _keypose_idx i) s)
       )
     ;; quadrotor_msgs/FtrPointArray[] _viewpoints_pos
     (write-long (length _viewpoints_pos) s)
     (dolist (elem _viewpoints_pos)
       (send elem :serialize s)
       )
     ;; std_msgs/Float32MultiArray[] _viewpoints_yaw
     (write-long (length _viewpoints_yaw) s)
     (dolist (elem _viewpoints_yaw)
       (send elem :serialize s)
       )
     ;; std_msgs/UInt16MultiArray[] _viewpoints_visib_num
     (write-long (length _viewpoints_visib_num) s)
     (dolist (elem _viewpoints_visib_num)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _box_min_
     (write-long (length _box_min_) s)
     (dolist (elem _box_min_)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _box_max_
     (write-long (length _box_max_) s)
     (dolist (elem _box_max_)
       (send elem :serialize s)
       )
     ;; quadrotor_msgs/FtrPathArray[] _paths
     (write-long (length _paths) s)
     (dolist (elem _paths)
       (send elem :serialize s)
       )
     ;; std_msgs/Float32MultiArray[] _costs
     (write-long (length _costs) s)
     (dolist (elem _costs)
       (send elem :serialize s)
       )
     ;; quadrotor_msgs/FtrPointArray[] _path_to_home_3
     (write-long (length _path_to_home_3) s)
     (dolist (elem _path_to_home_3)
       (send elem :serialize s)
       )
     ;; std_msgs/Float32MultiArray[] _path_to_home_4
     (write-long (length _path_to_home_4) s)
     (dolist (elem _path_to_home_4)
       (send elem :serialize s)
       )
     ;; float32[] _cost_to_home
     (write-long (length _cost_to_home) s)
     (dotimes (i (length _cost_to_home))
       (sys::poke (elt _cost_to_home i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; std_msgs/UInt16MultiArray[] _topo_blacklist
     (write-long (length _topo_blacklist) s)
     (dolist (elem _topo_blacklist)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _ftr_blacklist
     (write-long (length _ftr_blacklist) s)
     (dolist (elem _ftr_blacklist)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; quadrotor_msgs/FtrPointArray[] _cells
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cells (let (r) (dotimes (i n) (push (instance quadrotor_msgs::FtrPointArray :init) r)) r))
     (dolist (elem- _cells)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; quadrotor_msgs/FtrPointArray[] _filtered_cells
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _filtered_cells (let (r) (dotimes (i n) (push (instance quadrotor_msgs::FtrPointArray :init) r)) r))
     (dolist (elem- _filtered_cells)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _average
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _average (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _average)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _normal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _normal (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _normal)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint16[] _id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _id i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _keypose_idx
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keypose_idx (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _keypose_idx i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; quadrotor_msgs/FtrPointArray[] _viewpoints_pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _viewpoints_pos (let (r) (dotimes (i n) (push (instance quadrotor_msgs::FtrPointArray :init) r)) r))
     (dolist (elem- _viewpoints_pos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float32MultiArray[] _viewpoints_yaw
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _viewpoints_yaw (let (r) (dotimes (i n) (push (instance std_msgs::Float32MultiArray :init) r)) r))
     (dolist (elem- _viewpoints_yaw)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/UInt16MultiArray[] _viewpoints_visib_num
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _viewpoints_visib_num (let (r) (dotimes (i n) (push (instance std_msgs::UInt16MultiArray :init) r)) r))
     (dolist (elem- _viewpoints_visib_num)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _box_min_
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _box_min_ (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _box_min_)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _box_max_
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _box_max_ (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _box_max_)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; quadrotor_msgs/FtrPathArray[] _paths
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _paths (let (r) (dotimes (i n) (push (instance quadrotor_msgs::FtrPathArray :init) r)) r))
     (dolist (elem- _paths)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float32MultiArray[] _costs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _costs (let (r) (dotimes (i n) (push (instance std_msgs::Float32MultiArray :init) r)) r))
     (dolist (elem- _costs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; quadrotor_msgs/FtrPointArray[] _path_to_home_3
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path_to_home_3 (let (r) (dotimes (i n) (push (instance quadrotor_msgs::FtrPointArray :init) r)) r))
     (dolist (elem- _path_to_home_3)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float32MultiArray[] _path_to_home_4
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path_to_home_4 (let (r) (dotimes (i n) (push (instance std_msgs::Float32MultiArray :init) r)) r))
     (dolist (elem- _path_to_home_4)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _cost_to_home
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cost_to_home (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cost_to_home i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; std_msgs/UInt16MultiArray[] _topo_blacklist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _topo_blacklist (let (r) (dotimes (i n) (push (instance std_msgs::UInt16MultiArray :init) r)) r))
     (dolist (elem- _topo_blacklist)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _ftr_blacklist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ftr_blacklist (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _ftr_blacklist)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get quadrotor_msgs::FrontierMsg :md5sum-) "ca213f9395670f68c3f0d4e819744694")
(setf (get quadrotor_msgs::FrontierMsg :datatype-) "quadrotor_msgs/FrontierMsg")
(setf (get quadrotor_msgs::FrontierMsg :definition-)
      "Header header
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
MSG: quadrotor_msgs/FtrPointArray
geometry_msgs/Point[] PointArray
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


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
MSG: std_msgs/UInt16MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
uint16[]            data        # array of data


================================================================================
MSG: quadrotor_msgs/FtrPathArray
FtrPointArray[] PathArray

")



(provide :quadrotor_msgs/FrontierMsg "ca213f9395670f68c3f0d4e819744694")


