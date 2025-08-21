// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FtrPointArray = require('./FtrPointArray.js');
let FtrPathArray = require('./FtrPathArray.js');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class FrontierMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cells = null;
      this.filtered_cells = null;
      this.average = null;
      this.normal = null;
      this.id = null;
      this.keypose_idx = null;
      this.viewpoints_pos = null;
      this.viewpoints_yaw = null;
      this.viewpoints_visib_num = null;
      this.box_min_ = null;
      this.box_max_ = null;
      this.paths = null;
      this.costs = null;
      this.path_to_home_3 = null;
      this.path_to_home_4 = null;
      this.cost_to_home = null;
      this.topo_blacklist = null;
      this.ftr_blacklist = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cells')) {
        this.cells = initObj.cells
      }
      else {
        this.cells = [];
      }
      if (initObj.hasOwnProperty('filtered_cells')) {
        this.filtered_cells = initObj.filtered_cells
      }
      else {
        this.filtered_cells = [];
      }
      if (initObj.hasOwnProperty('average')) {
        this.average = initObj.average
      }
      else {
        this.average = [];
      }
      if (initObj.hasOwnProperty('normal')) {
        this.normal = initObj.normal
      }
      else {
        this.normal = [];
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = [];
      }
      if (initObj.hasOwnProperty('keypose_idx')) {
        this.keypose_idx = initObj.keypose_idx
      }
      else {
        this.keypose_idx = [];
      }
      if (initObj.hasOwnProperty('viewpoints_pos')) {
        this.viewpoints_pos = initObj.viewpoints_pos
      }
      else {
        this.viewpoints_pos = [];
      }
      if (initObj.hasOwnProperty('viewpoints_yaw')) {
        this.viewpoints_yaw = initObj.viewpoints_yaw
      }
      else {
        this.viewpoints_yaw = [];
      }
      if (initObj.hasOwnProperty('viewpoints_visib_num')) {
        this.viewpoints_visib_num = initObj.viewpoints_visib_num
      }
      else {
        this.viewpoints_visib_num = [];
      }
      if (initObj.hasOwnProperty('box_min_')) {
        this.box_min_ = initObj.box_min_
      }
      else {
        this.box_min_ = [];
      }
      if (initObj.hasOwnProperty('box_max_')) {
        this.box_max_ = initObj.box_max_
      }
      else {
        this.box_max_ = [];
      }
      if (initObj.hasOwnProperty('paths')) {
        this.paths = initObj.paths
      }
      else {
        this.paths = [];
      }
      if (initObj.hasOwnProperty('costs')) {
        this.costs = initObj.costs
      }
      else {
        this.costs = [];
      }
      if (initObj.hasOwnProperty('path_to_home_3')) {
        this.path_to_home_3 = initObj.path_to_home_3
      }
      else {
        this.path_to_home_3 = [];
      }
      if (initObj.hasOwnProperty('path_to_home_4')) {
        this.path_to_home_4 = initObj.path_to_home_4
      }
      else {
        this.path_to_home_4 = [];
      }
      if (initObj.hasOwnProperty('cost_to_home')) {
        this.cost_to_home = initObj.cost_to_home
      }
      else {
        this.cost_to_home = [];
      }
      if (initObj.hasOwnProperty('topo_blacklist')) {
        this.topo_blacklist = initObj.topo_blacklist
      }
      else {
        this.topo_blacklist = [];
      }
      if (initObj.hasOwnProperty('ftr_blacklist')) {
        this.ftr_blacklist = initObj.ftr_blacklist
      }
      else {
        this.ftr_blacklist = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrontierMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cells]
    // Serialize the length for message field [cells]
    bufferOffset = _serializer.uint32(obj.cells.length, buffer, bufferOffset);
    obj.cells.forEach((val) => {
      bufferOffset = FtrPointArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [filtered_cells]
    // Serialize the length for message field [filtered_cells]
    bufferOffset = _serializer.uint32(obj.filtered_cells.length, buffer, bufferOffset);
    obj.filtered_cells.forEach((val) => {
      bufferOffset = FtrPointArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [average]
    // Serialize the length for message field [average]
    bufferOffset = _serializer.uint32(obj.average.length, buffer, bufferOffset);
    obj.average.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [normal]
    // Serialize the length for message field [normal]
    bufferOffset = _serializer.uint32(obj.normal.length, buffer, bufferOffset);
    obj.normal.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [id]
    bufferOffset = _arraySerializer.uint16(obj.id, buffer, bufferOffset, null);
    // Serialize message field [keypose_idx]
    bufferOffset = _arraySerializer.uint16(obj.keypose_idx, buffer, bufferOffset, null);
    // Serialize message field [viewpoints_pos]
    // Serialize the length for message field [viewpoints_pos]
    bufferOffset = _serializer.uint32(obj.viewpoints_pos.length, buffer, bufferOffset);
    obj.viewpoints_pos.forEach((val) => {
      bufferOffset = FtrPointArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [viewpoints_yaw]
    // Serialize the length for message field [viewpoints_yaw]
    bufferOffset = _serializer.uint32(obj.viewpoints_yaw.length, buffer, bufferOffset);
    obj.viewpoints_yaw.forEach((val) => {
      bufferOffset = std_msgs.msg.Float32MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [viewpoints_visib_num]
    // Serialize the length for message field [viewpoints_visib_num]
    bufferOffset = _serializer.uint32(obj.viewpoints_visib_num.length, buffer, bufferOffset);
    obj.viewpoints_visib_num.forEach((val) => {
      bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [box_min_]
    // Serialize the length for message field [box_min_]
    bufferOffset = _serializer.uint32(obj.box_min_.length, buffer, bufferOffset);
    obj.box_min_.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [box_max_]
    // Serialize the length for message field [box_max_]
    bufferOffset = _serializer.uint32(obj.box_max_.length, buffer, bufferOffset);
    obj.box_max_.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [paths]
    // Serialize the length for message field [paths]
    bufferOffset = _serializer.uint32(obj.paths.length, buffer, bufferOffset);
    obj.paths.forEach((val) => {
      bufferOffset = FtrPathArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [costs]
    // Serialize the length for message field [costs]
    bufferOffset = _serializer.uint32(obj.costs.length, buffer, bufferOffset);
    obj.costs.forEach((val) => {
      bufferOffset = std_msgs.msg.Float32MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [path_to_home_3]
    // Serialize the length for message field [path_to_home_3]
    bufferOffset = _serializer.uint32(obj.path_to_home_3.length, buffer, bufferOffset);
    obj.path_to_home_3.forEach((val) => {
      bufferOffset = FtrPointArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [path_to_home_4]
    // Serialize the length for message field [path_to_home_4]
    bufferOffset = _serializer.uint32(obj.path_to_home_4.length, buffer, bufferOffset);
    obj.path_to_home_4.forEach((val) => {
      bufferOffset = std_msgs.msg.Float32MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cost_to_home]
    bufferOffset = _arraySerializer.float32(obj.cost_to_home, buffer, bufferOffset, null);
    // Serialize message field [topo_blacklist]
    // Serialize the length for message field [topo_blacklist]
    bufferOffset = _serializer.uint32(obj.topo_blacklist.length, buffer, bufferOffset);
    obj.topo_blacklist.forEach((val) => {
      bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [ftr_blacklist]
    // Serialize the length for message field [ftr_blacklist]
    bufferOffset = _serializer.uint32(obj.ftr_blacklist.length, buffer, bufferOffset);
    obj.ftr_blacklist.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrontierMsg
    let len;
    let data = new FrontierMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cells]
    // Deserialize array length for message field [cells]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cells = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cells[i] = FtrPointArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [filtered_cells]
    // Deserialize array length for message field [filtered_cells]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.filtered_cells = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.filtered_cells[i] = FtrPointArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [average]
    // Deserialize array length for message field [average]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.average = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.average[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [normal]
    // Deserialize array length for message field [normal]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.normal = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.normal[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [id]
    data.id = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [keypose_idx]
    data.keypose_idx = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [viewpoints_pos]
    // Deserialize array length for message field [viewpoints_pos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.viewpoints_pos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.viewpoints_pos[i] = FtrPointArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [viewpoints_yaw]
    // Deserialize array length for message field [viewpoints_yaw]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.viewpoints_yaw = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.viewpoints_yaw[i] = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [viewpoints_visib_num]
    // Deserialize array length for message field [viewpoints_visib_num]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.viewpoints_visib_num = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.viewpoints_visib_num[i] = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [box_min_]
    // Deserialize array length for message field [box_min_]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.box_min_ = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.box_min_[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [box_max_]
    // Deserialize array length for message field [box_max_]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.box_max_ = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.box_max_[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [paths]
    // Deserialize array length for message field [paths]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.paths = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.paths[i] = FtrPathArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [costs]
    // Deserialize array length for message field [costs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.costs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.costs[i] = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [path_to_home_3]
    // Deserialize array length for message field [path_to_home_3]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path_to_home_3 = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path_to_home_3[i] = FtrPointArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [path_to_home_4]
    // Deserialize array length for message field [path_to_home_4]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path_to_home_4 = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path_to_home_4[i] = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cost_to_home]
    data.cost_to_home = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [topo_blacklist]
    // Deserialize array length for message field [topo_blacklist]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.topo_blacklist = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.topo_blacklist[i] = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [ftr_blacklist]
    // Deserialize array length for message field [ftr_blacklist]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ftr_blacklist = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ftr_blacklist[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.cells.forEach((val) => {
      length += FtrPointArray.getMessageSize(val);
    });
    object.filtered_cells.forEach((val) => {
      length += FtrPointArray.getMessageSize(val);
    });
    length += 24 * object.average.length;
    length += 24 * object.normal.length;
    length += 2 * object.id.length;
    length += 2 * object.keypose_idx.length;
    object.viewpoints_pos.forEach((val) => {
      length += FtrPointArray.getMessageSize(val);
    });
    object.viewpoints_yaw.forEach((val) => {
      length += std_msgs.msg.Float32MultiArray.getMessageSize(val);
    });
    object.viewpoints_visib_num.forEach((val) => {
      length += std_msgs.msg.UInt16MultiArray.getMessageSize(val);
    });
    length += 24 * object.box_min_.length;
    length += 24 * object.box_max_.length;
    object.paths.forEach((val) => {
      length += FtrPathArray.getMessageSize(val);
    });
    object.costs.forEach((val) => {
      length += std_msgs.msg.Float32MultiArray.getMessageSize(val);
    });
    object.path_to_home_3.forEach((val) => {
      length += FtrPointArray.getMessageSize(val);
    });
    object.path_to_home_4.forEach((val) => {
      length += std_msgs.msg.Float32MultiArray.getMessageSize(val);
    });
    length += 4 * object.cost_to_home.length;
    object.topo_blacklist.forEach((val) => {
      length += std_msgs.msg.UInt16MultiArray.getMessageSize(val);
    });
    length += 24 * object.ftr_blacklist.length;
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/FrontierMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca213f9395670f68c3f0d4e819744694';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    MSG: std_msgs/UInt16MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    uint16[]            data        # array of data
    
    
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
    const resolved = new FrontierMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cells !== undefined) {
      resolved.cells = new Array(msg.cells.length);
      for (let i = 0; i < resolved.cells.length; ++i) {
        resolved.cells[i] = FtrPointArray.Resolve(msg.cells[i]);
      }
    }
    else {
      resolved.cells = []
    }

    if (msg.filtered_cells !== undefined) {
      resolved.filtered_cells = new Array(msg.filtered_cells.length);
      for (let i = 0; i < resolved.filtered_cells.length; ++i) {
        resolved.filtered_cells[i] = FtrPointArray.Resolve(msg.filtered_cells[i]);
      }
    }
    else {
      resolved.filtered_cells = []
    }

    if (msg.average !== undefined) {
      resolved.average = new Array(msg.average.length);
      for (let i = 0; i < resolved.average.length; ++i) {
        resolved.average[i] = geometry_msgs.msg.Point.Resolve(msg.average[i]);
      }
    }
    else {
      resolved.average = []
    }

    if (msg.normal !== undefined) {
      resolved.normal = new Array(msg.normal.length);
      for (let i = 0; i < resolved.normal.length; ++i) {
        resolved.normal[i] = geometry_msgs.msg.Point.Resolve(msg.normal[i]);
      }
    }
    else {
      resolved.normal = []
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = []
    }

    if (msg.keypose_idx !== undefined) {
      resolved.keypose_idx = msg.keypose_idx;
    }
    else {
      resolved.keypose_idx = []
    }

    if (msg.viewpoints_pos !== undefined) {
      resolved.viewpoints_pos = new Array(msg.viewpoints_pos.length);
      for (let i = 0; i < resolved.viewpoints_pos.length; ++i) {
        resolved.viewpoints_pos[i] = FtrPointArray.Resolve(msg.viewpoints_pos[i]);
      }
    }
    else {
      resolved.viewpoints_pos = []
    }

    if (msg.viewpoints_yaw !== undefined) {
      resolved.viewpoints_yaw = new Array(msg.viewpoints_yaw.length);
      for (let i = 0; i < resolved.viewpoints_yaw.length; ++i) {
        resolved.viewpoints_yaw[i] = std_msgs.msg.Float32MultiArray.Resolve(msg.viewpoints_yaw[i]);
      }
    }
    else {
      resolved.viewpoints_yaw = []
    }

    if (msg.viewpoints_visib_num !== undefined) {
      resolved.viewpoints_visib_num = new Array(msg.viewpoints_visib_num.length);
      for (let i = 0; i < resolved.viewpoints_visib_num.length; ++i) {
        resolved.viewpoints_visib_num[i] = std_msgs.msg.UInt16MultiArray.Resolve(msg.viewpoints_visib_num[i]);
      }
    }
    else {
      resolved.viewpoints_visib_num = []
    }

    if (msg.box_min_ !== undefined) {
      resolved.box_min_ = new Array(msg.box_min_.length);
      for (let i = 0; i < resolved.box_min_.length; ++i) {
        resolved.box_min_[i] = geometry_msgs.msg.Point.Resolve(msg.box_min_[i]);
      }
    }
    else {
      resolved.box_min_ = []
    }

    if (msg.box_max_ !== undefined) {
      resolved.box_max_ = new Array(msg.box_max_.length);
      for (let i = 0; i < resolved.box_max_.length; ++i) {
        resolved.box_max_[i] = geometry_msgs.msg.Point.Resolve(msg.box_max_[i]);
      }
    }
    else {
      resolved.box_max_ = []
    }

    if (msg.paths !== undefined) {
      resolved.paths = new Array(msg.paths.length);
      for (let i = 0; i < resolved.paths.length; ++i) {
        resolved.paths[i] = FtrPathArray.Resolve(msg.paths[i]);
      }
    }
    else {
      resolved.paths = []
    }

    if (msg.costs !== undefined) {
      resolved.costs = new Array(msg.costs.length);
      for (let i = 0; i < resolved.costs.length; ++i) {
        resolved.costs[i] = std_msgs.msg.Float32MultiArray.Resolve(msg.costs[i]);
      }
    }
    else {
      resolved.costs = []
    }

    if (msg.path_to_home_3 !== undefined) {
      resolved.path_to_home_3 = new Array(msg.path_to_home_3.length);
      for (let i = 0; i < resolved.path_to_home_3.length; ++i) {
        resolved.path_to_home_3[i] = FtrPointArray.Resolve(msg.path_to_home_3[i]);
      }
    }
    else {
      resolved.path_to_home_3 = []
    }

    if (msg.path_to_home_4 !== undefined) {
      resolved.path_to_home_4 = new Array(msg.path_to_home_4.length);
      for (let i = 0; i < resolved.path_to_home_4.length; ++i) {
        resolved.path_to_home_4[i] = std_msgs.msg.Float32MultiArray.Resolve(msg.path_to_home_4[i]);
      }
    }
    else {
      resolved.path_to_home_4 = []
    }

    if (msg.cost_to_home !== undefined) {
      resolved.cost_to_home = msg.cost_to_home;
    }
    else {
      resolved.cost_to_home = []
    }

    if (msg.topo_blacklist !== undefined) {
      resolved.topo_blacklist = new Array(msg.topo_blacklist.length);
      for (let i = 0; i < resolved.topo_blacklist.length; ++i) {
        resolved.topo_blacklist[i] = std_msgs.msg.UInt16MultiArray.Resolve(msg.topo_blacklist[i]);
      }
    }
    else {
      resolved.topo_blacklist = []
    }

    if (msg.ftr_blacklist !== undefined) {
      resolved.ftr_blacklist = new Array(msg.ftr_blacklist.length);
      for (let i = 0; i < resolved.ftr_blacklist.length; ++i) {
        resolved.ftr_blacklist[i] = geometry_msgs.msg.Point.Resolve(msg.ftr_blacklist[i]);
      }
    }
    else {
      resolved.ftr_blacklist = []
    }

    return resolved;
    }
};

module.exports = FrontierMsg;
