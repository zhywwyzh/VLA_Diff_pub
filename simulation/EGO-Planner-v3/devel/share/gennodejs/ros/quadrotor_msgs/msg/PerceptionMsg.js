// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MultiPoseGraph = require('./MultiPoseGraph.js');
let HgridMsg = require('./HgridMsg.js');
let FrontierMsg = require('./FrontierMsg.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PerceptionMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.from_drone_id = null;
      this.to_drone_id = null;
      this.req_or_resp = null;
      this.msg_type = null;
      this.posegraph_msg = null;
      this.hgrid_msg = null;
      this.ftr_msg = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('from_drone_id')) {
        this.from_drone_id = initObj.from_drone_id
      }
      else {
        this.from_drone_id = 0;
      }
      if (initObj.hasOwnProperty('to_drone_id')) {
        this.to_drone_id = initObj.to_drone_id
      }
      else {
        this.to_drone_id = 0;
      }
      if (initObj.hasOwnProperty('req_or_resp')) {
        this.req_or_resp = initObj.req_or_resp
      }
      else {
        this.req_or_resp = 0;
      }
      if (initObj.hasOwnProperty('msg_type')) {
        this.msg_type = initObj.msg_type
      }
      else {
        this.msg_type = 0;
      }
      if (initObj.hasOwnProperty('posegraph_msg')) {
        this.posegraph_msg = initObj.posegraph_msg
      }
      else {
        this.posegraph_msg = new MultiPoseGraph();
      }
      if (initObj.hasOwnProperty('hgrid_msg')) {
        this.hgrid_msg = initObj.hgrid_msg
      }
      else {
        this.hgrid_msg = new HgridMsg();
      }
      if (initObj.hasOwnProperty('ftr_msg')) {
        this.ftr_msg = initObj.ftr_msg
      }
      else {
        this.ftr_msg = new FrontierMsg();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PerceptionMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [from_drone_id]
    bufferOffset = _serializer.uint16(obj.from_drone_id, buffer, bufferOffset);
    // Serialize message field [to_drone_id]
    bufferOffset = _serializer.uint16(obj.to_drone_id, buffer, bufferOffset);
    // Serialize message field [req_or_resp]
    bufferOffset = _serializer.uint8(obj.req_or_resp, buffer, bufferOffset);
    // Serialize message field [msg_type]
    bufferOffset = _serializer.uint8(obj.msg_type, buffer, bufferOffset);
    // Serialize message field [posegraph_msg]
    bufferOffset = MultiPoseGraph.serialize(obj.posegraph_msg, buffer, bufferOffset);
    // Serialize message field [hgrid_msg]
    bufferOffset = HgridMsg.serialize(obj.hgrid_msg, buffer, bufferOffset);
    // Serialize message field [ftr_msg]
    bufferOffset = FrontierMsg.serialize(obj.ftr_msg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PerceptionMsg
    let len;
    let data = new PerceptionMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [from_drone_id]
    data.from_drone_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [to_drone_id]
    data.to_drone_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [req_or_resp]
    data.req_or_resp = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [msg_type]
    data.msg_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [posegraph_msg]
    data.posegraph_msg = MultiPoseGraph.deserialize(buffer, bufferOffset);
    // Deserialize message field [hgrid_msg]
    data.hgrid_msg = HgridMsg.deserialize(buffer, bufferOffset);
    // Deserialize message field [ftr_msg]
    data.ftr_msg = FrontierMsg.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += MultiPoseGraph.getMessageSize(object.posegraph_msg);
    length += HgridMsg.getMessageSize(object.hgrid_msg);
    length += FrontierMsg.getMessageSize(object.ftr_msg);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/PerceptionMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1dcaf3ff09d27642f7d2a9908eb37ead';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PerceptionMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.from_drone_id !== undefined) {
      resolved.from_drone_id = msg.from_drone_id;
    }
    else {
      resolved.from_drone_id = 0
    }

    if (msg.to_drone_id !== undefined) {
      resolved.to_drone_id = msg.to_drone_id;
    }
    else {
      resolved.to_drone_id = 0
    }

    if (msg.req_or_resp !== undefined) {
      resolved.req_or_resp = msg.req_or_resp;
    }
    else {
      resolved.req_or_resp = 0
    }

    if (msg.msg_type !== undefined) {
      resolved.msg_type = msg.msg_type;
    }
    else {
      resolved.msg_type = 0
    }

    if (msg.posegraph_msg !== undefined) {
      resolved.posegraph_msg = MultiPoseGraph.Resolve(msg.posegraph_msg)
    }
    else {
      resolved.posegraph_msg = new MultiPoseGraph()
    }

    if (msg.hgrid_msg !== undefined) {
      resolved.hgrid_msg = HgridMsg.Resolve(msg.hgrid_msg)
    }
    else {
      resolved.hgrid_msg = new HgridMsg()
    }

    if (msg.ftr_msg !== undefined) {
      resolved.ftr_msg = FrontierMsg.Resolve(msg.ftr_msg)
    }
    else {
      resolved.ftr_msg = new FrontierMsg()
    }

    return resolved;
    }
};

// Constants for message
PerceptionMsg.Constants = {
  DATA_NEED_MAP_MERGE: 1,
  DATA_NEED_MAP_RESET: 2,
  RESPONSE_MSG_FALG: 2,
  REQUESET_MSG_FLAG: 1,
}

module.exports = PerceptionMsg;
