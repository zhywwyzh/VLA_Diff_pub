;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::HgridMsg)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'HgridMsg (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::HGRIDMSG")
  (make-package "QUADROTOR_MSGS::HGRIDMSG"))

(in-package "ROS")
;;//! \htmlinclude HgridMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass quadrotor_msgs::HgridMsg
  :super ros::object
  :slots (_header _recv _id _local_id _unknown_num _frontier_num _center _frontier_cell_nums _contained_frontier_ids _is_updated _need_divide _active _is_prev_relevant _is_cur_relevant _is_covered _relevant_id _relevant_map _min _max _resolution _grid_size _multi_layer_hgrid ))

(defmethod quadrotor_msgs::HgridMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:recv __recv) nil)
    ((:id __id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:local_id __local_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:unknown_num __unknown_num) (make-array 0 :initial-element 0 :element-type :integer))
    ((:frontier_num __frontier_num) (make-array 0 :initial-element 0 :element-type :integer))
    ((:center __center) ())
    ((:frontier_cell_nums __frontier_cell_nums) ())
    ((:contained_frontier_ids __contained_frontier_ids) ())
    ((:is_updated __is_updated) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:need_divide __need_divide) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:active __active) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:is_prev_relevant __is_prev_relevant) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:is_cur_relevant __is_cur_relevant) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:is_covered __is_covered) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:relevant_id __relevant_id) (instance std_msgs::UInt16MultiArray :init))
    ((:relevant_map __relevant_map) (instance std_msgs::UInt16MultiArray :init))
    ((:min __min) (instance geometry_msgs::Point :init))
    ((:max __max) (instance geometry_msgs::Point :init))
    ((:resolution __resolution) (instance geometry_msgs::Point :init))
    ((:grid_size __grid_size) 0.0)
    ((:multi_layer_hgrid __multi_layer_hgrid) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _recv __recv)
   (setq _id __id)
   (setq _local_id __local_id)
   (setq _unknown_num __unknown_num)
   (setq _frontier_num __frontier_num)
   (setq _center __center)
   (setq _frontier_cell_nums __frontier_cell_nums)
   (setq _contained_frontier_ids __contained_frontier_ids)
   (setq _is_updated __is_updated)
   (setq _need_divide __need_divide)
   (setq _active __active)
   (setq _is_prev_relevant __is_prev_relevant)
   (setq _is_cur_relevant __is_cur_relevant)
   (setq _is_covered __is_covered)
   (setq _relevant_id __relevant_id)
   (setq _relevant_map __relevant_map)
   (setq _min __min)
   (setq _max __max)
   (setq _resolution __resolution)
   (setq _grid_size (float __grid_size))
   (setq _multi_layer_hgrid __multi_layer_hgrid)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:recv
   (&optional (__recv :null))
   (if (not (eq __recv :null)) (setq _recv __recv)) _recv)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:local_id
   (&optional __local_id)
   (if __local_id (setq _local_id __local_id)) _local_id)
  (:unknown_num
   (&optional __unknown_num)
   (if __unknown_num (setq _unknown_num __unknown_num)) _unknown_num)
  (:frontier_num
   (&optional __frontier_num)
   (if __frontier_num (setq _frontier_num __frontier_num)) _frontier_num)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:frontier_cell_nums
   (&rest __frontier_cell_nums)
   (if (keywordp (car __frontier_cell_nums))
       (send* _frontier_cell_nums __frontier_cell_nums)
     (progn
       (if __frontier_cell_nums (setq _frontier_cell_nums (car __frontier_cell_nums)))
       _frontier_cell_nums)))
  (:contained_frontier_ids
   (&rest __contained_frontier_ids)
   (if (keywordp (car __contained_frontier_ids))
       (send* _contained_frontier_ids __contained_frontier_ids)
     (progn
       (if __contained_frontier_ids (setq _contained_frontier_ids (car __contained_frontier_ids)))
       _contained_frontier_ids)))
  (:is_updated
   (&optional __is_updated)
   (if __is_updated (setq _is_updated __is_updated)) _is_updated)
  (:need_divide
   (&optional __need_divide)
   (if __need_divide (setq _need_divide __need_divide)) _need_divide)
  (:active
   (&optional __active)
   (if __active (setq _active __active)) _active)
  (:is_prev_relevant
   (&optional __is_prev_relevant)
   (if __is_prev_relevant (setq _is_prev_relevant __is_prev_relevant)) _is_prev_relevant)
  (:is_cur_relevant
   (&optional __is_cur_relevant)
   (if __is_cur_relevant (setq _is_cur_relevant __is_cur_relevant)) _is_cur_relevant)
  (:is_covered
   (&optional __is_covered)
   (if __is_covered (setq _is_covered __is_covered)) _is_covered)
  (:relevant_id
   (&rest __relevant_id)
   (if (keywordp (car __relevant_id))
       (send* _relevant_id __relevant_id)
     (progn
       (if __relevant_id (setq _relevant_id (car __relevant_id)))
       _relevant_id)))
  (:relevant_map
   (&rest __relevant_map)
   (if (keywordp (car __relevant_map))
       (send* _relevant_map __relevant_map)
     (progn
       (if __relevant_map (setq _relevant_map (car __relevant_map)))
       _relevant_map)))
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:resolution
   (&rest __resolution)
   (if (keywordp (car __resolution))
       (send* _resolution __resolution)
     (progn
       (if __resolution (setq _resolution (car __resolution)))
       _resolution)))
  (:grid_size
   (&optional __grid_size)
   (if __grid_size (setq _grid_size __grid_size)) _grid_size)
  (:multi_layer_hgrid
   (&optional (__multi_layer_hgrid :null))
   (if (not (eq __multi_layer_hgrid :null)) (setq _multi_layer_hgrid __multi_layer_hgrid)) _multi_layer_hgrid)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _recv
    1
    ;; uint16[] _id
    (* 2    (length _id)) 4
    ;; uint16[] _local_id
    (* 2    (length _local_id)) 4
    ;; uint16[] _unknown_num
    (* 2    (length _unknown_num)) 4
    ;; uint16[] _frontier_num
    (* 2    (length _frontier_num)) 4
    ;; geometry_msgs/Point[] _center
    (apply #'+ (send-all _center :serialization-length)) 4
    ;; std_msgs/UInt16MultiArray[] _frontier_cell_nums
    (apply #'+ (send-all _frontier_cell_nums :serialization-length)) 4
    ;; std_msgs/UInt16MultiArray[] _contained_frontier_ids
    (apply #'+ (send-all _contained_frontier_ids :serialization-length)) 4
    ;; bool[] _is_updated
    (* 1    (length _is_updated)) 4
    ;; bool[] _need_divide
    (* 1    (length _need_divide)) 4
    ;; bool[] _active
    (* 1    (length _active)) 4
    ;; bool[] _is_prev_relevant
    (* 1    (length _is_prev_relevant)) 4
    ;; bool[] _is_cur_relevant
    (* 1    (length _is_cur_relevant)) 4
    ;; bool[] _is_covered
    (* 1    (length _is_covered)) 4
    ;; std_msgs/UInt16MultiArray _relevant_id
    (send _relevant_id :serialization-length)
    ;; std_msgs/UInt16MultiArray _relevant_map
    (send _relevant_map :serialization-length)
    ;; geometry_msgs/Point _min
    (send _min :serialization-length)
    ;; geometry_msgs/Point _max
    (send _max :serialization-length)
    ;; geometry_msgs/Point _resolution
    (send _resolution :serialization-length)
    ;; float32 _grid_size
    4
    ;; bool _multi_layer_hgrid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _recv
       (if _recv (write-byte -1 s) (write-byte 0 s))
     ;; uint16[] _id
     (write-long (length _id) s)
     (dotimes (i (length _id))
       (write-word (elt _id i) s)
       )
     ;; uint16[] _local_id
     (write-long (length _local_id) s)
     (dotimes (i (length _local_id))
       (write-word (elt _local_id i) s)
       )
     ;; uint16[] _unknown_num
     (write-long (length _unknown_num) s)
     (dotimes (i (length _unknown_num))
       (write-word (elt _unknown_num i) s)
       )
     ;; uint16[] _frontier_num
     (write-long (length _frontier_num) s)
     (dotimes (i (length _frontier_num))
       (write-word (elt _frontier_num i) s)
       )
     ;; geometry_msgs/Point[] _center
     (write-long (length _center) s)
     (dolist (elem _center)
       (send elem :serialize s)
       )
     ;; std_msgs/UInt16MultiArray[] _frontier_cell_nums
     (write-long (length _frontier_cell_nums) s)
     (dolist (elem _frontier_cell_nums)
       (send elem :serialize s)
       )
     ;; std_msgs/UInt16MultiArray[] _contained_frontier_ids
     (write-long (length _contained_frontier_ids) s)
     (dolist (elem _contained_frontier_ids)
       (send elem :serialize s)
       )
     ;; bool[] _is_updated
     (write-long (length _is_updated) s)
     (dotimes (i (length _is_updated))
       (if (elt _is_updated i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[] _need_divide
     (write-long (length _need_divide) s)
     (dotimes (i (length _need_divide))
       (if (elt _need_divide i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[] _active
     (write-long (length _active) s)
     (dotimes (i (length _active))
       (if (elt _active i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[] _is_prev_relevant
     (write-long (length _is_prev_relevant) s)
     (dotimes (i (length _is_prev_relevant))
       (if (elt _is_prev_relevant i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[] _is_cur_relevant
     (write-long (length _is_cur_relevant) s)
     (dotimes (i (length _is_cur_relevant))
       (if (elt _is_cur_relevant i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[] _is_covered
     (write-long (length _is_covered) s)
     (dotimes (i (length _is_covered))
       (if (elt _is_covered i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; std_msgs/UInt16MultiArray _relevant_id
       (send _relevant_id :serialize s)
     ;; std_msgs/UInt16MultiArray _relevant_map
       (send _relevant_map :serialize s)
     ;; geometry_msgs/Point _min
       (send _min :serialize s)
     ;; geometry_msgs/Point _max
       (send _max :serialize s)
     ;; geometry_msgs/Point _resolution
       (send _resolution :serialize s)
     ;; float32 _grid_size
       (sys::poke _grid_size (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _multi_layer_hgrid
       (if _multi_layer_hgrid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _recv
     (setq _recv (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16[] _id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _id i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _local_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _local_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _local_id i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _unknown_num
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _unknown_num (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _unknown_num i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _frontier_num
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frontier_num (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _frontier_num i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; geometry_msgs/Point[] _center
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _center (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _center)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/UInt16MultiArray[] _frontier_cell_nums
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frontier_cell_nums (let (r) (dotimes (i n) (push (instance std_msgs::UInt16MultiArray :init) r)) r))
     (dolist (elem- _frontier_cell_nums)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/UInt16MultiArray[] _contained_frontier_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _contained_frontier_ids (let (r) (dotimes (i n) (push (instance std_msgs::UInt16MultiArray :init) r)) r))
     (dolist (elem- _contained_frontier_ids)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool[] _is_updated
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _is_updated (make-list n))
     (dotimes (i n)
     (setf (elt _is_updated i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; bool[] _need_divide
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _need_divide (make-list n))
     (dotimes (i n)
     (setf (elt _need_divide i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; bool[] _active
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _active (make-list n))
     (dotimes (i n)
     (setf (elt _active i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; bool[] _is_prev_relevant
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _is_prev_relevant (make-list n))
     (dotimes (i n)
     (setf (elt _is_prev_relevant i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; bool[] _is_cur_relevant
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _is_cur_relevant (make-list n))
     (dotimes (i n)
     (setf (elt _is_cur_relevant i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; bool[] _is_covered
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _is_covered (make-list n))
     (dotimes (i n)
     (setf (elt _is_covered i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; std_msgs/UInt16MultiArray _relevant_id
     (send _relevant_id :deserialize buf ptr-) (incf ptr- (send _relevant_id :serialization-length))
   ;; std_msgs/UInt16MultiArray _relevant_map
     (send _relevant_map :deserialize buf ptr-) (incf ptr- (send _relevant_map :serialization-length))
   ;; geometry_msgs/Point _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; geometry_msgs/Point _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;; geometry_msgs/Point _resolution
     (send _resolution :deserialize buf ptr-) (incf ptr- (send _resolution :serialization-length))
   ;; float32 _grid_size
     (setq _grid_size (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _multi_layer_hgrid
     (setq _multi_layer_hgrid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get quadrotor_msgs::HgridMsg :md5sum-) "d287d82d0713c1a695d6c48be1e56cc3")
(setf (get quadrotor_msgs::HgridMsg :datatype-) "quadrotor_msgs/HgridMsg")
(setf (get quadrotor_msgs::HgridMsg :definition-)
      "Header header
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
")



(provide :quadrotor_msgs/HgridMsg "d287d82d0713c1a695d6c48be1e56cc3")


