// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class HgridMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.recv = null;
      this.id = null;
      this.local_id = null;
      this.unknown_num = null;
      this.frontier_num = null;
      this.center = null;
      this.frontier_cell_nums = null;
      this.contained_frontier_ids = null;
      this.is_updated = null;
      this.need_divide = null;
      this.active = null;
      this.is_prev_relevant = null;
      this.is_cur_relevant = null;
      this.is_covered = null;
      this.relevant_id = null;
      this.relevant_map = null;
      this.min = null;
      this.max = null;
      this.resolution = null;
      this.grid_size = null;
      this.multi_layer_hgrid = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('recv')) {
        this.recv = initObj.recv
      }
      else {
        this.recv = false;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = [];
      }
      if (initObj.hasOwnProperty('local_id')) {
        this.local_id = initObj.local_id
      }
      else {
        this.local_id = [];
      }
      if (initObj.hasOwnProperty('unknown_num')) {
        this.unknown_num = initObj.unknown_num
      }
      else {
        this.unknown_num = [];
      }
      if (initObj.hasOwnProperty('frontier_num')) {
        this.frontier_num = initObj.frontier_num
      }
      else {
        this.frontier_num = [];
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = [];
      }
      if (initObj.hasOwnProperty('frontier_cell_nums')) {
        this.frontier_cell_nums = initObj.frontier_cell_nums
      }
      else {
        this.frontier_cell_nums = [];
      }
      if (initObj.hasOwnProperty('contained_frontier_ids')) {
        this.contained_frontier_ids = initObj.contained_frontier_ids
      }
      else {
        this.contained_frontier_ids = [];
      }
      if (initObj.hasOwnProperty('is_updated')) {
        this.is_updated = initObj.is_updated
      }
      else {
        this.is_updated = [];
      }
      if (initObj.hasOwnProperty('need_divide')) {
        this.need_divide = initObj.need_divide
      }
      else {
        this.need_divide = [];
      }
      if (initObj.hasOwnProperty('active')) {
        this.active = initObj.active
      }
      else {
        this.active = [];
      }
      if (initObj.hasOwnProperty('is_prev_relevant')) {
        this.is_prev_relevant = initObj.is_prev_relevant
      }
      else {
        this.is_prev_relevant = [];
      }
      if (initObj.hasOwnProperty('is_cur_relevant')) {
        this.is_cur_relevant = initObj.is_cur_relevant
      }
      else {
        this.is_cur_relevant = [];
      }
      if (initObj.hasOwnProperty('is_covered')) {
        this.is_covered = initObj.is_covered
      }
      else {
        this.is_covered = [];
      }
      if (initObj.hasOwnProperty('relevant_id')) {
        this.relevant_id = initObj.relevant_id
      }
      else {
        this.relevant_id = new std_msgs.msg.UInt16MultiArray();
      }
      if (initObj.hasOwnProperty('relevant_map')) {
        this.relevant_map = initObj.relevant_map
      }
      else {
        this.relevant_map = new std_msgs.msg.UInt16MultiArray();
      }
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('grid_size')) {
        this.grid_size = initObj.grid_size
      }
      else {
        this.grid_size = 0.0;
      }
      if (initObj.hasOwnProperty('multi_layer_hgrid')) {
        this.multi_layer_hgrid = initObj.multi_layer_hgrid
      }
      else {
        this.multi_layer_hgrid = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HgridMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [recv]
    bufferOffset = _serializer.bool(obj.recv, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _arraySerializer.uint16(obj.id, buffer, bufferOffset, null);
    // Serialize message field [local_id]
    bufferOffset = _arraySerializer.uint16(obj.local_id, buffer, bufferOffset, null);
    // Serialize message field [unknown_num]
    bufferOffset = _arraySerializer.uint16(obj.unknown_num, buffer, bufferOffset, null);
    // Serialize message field [frontier_num]
    bufferOffset = _arraySerializer.uint16(obj.frontier_num, buffer, bufferOffset, null);
    // Serialize message field [center]
    // Serialize the length for message field [center]
    bufferOffset = _serializer.uint32(obj.center.length, buffer, bufferOffset);
    obj.center.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [frontier_cell_nums]
    // Serialize the length for message field [frontier_cell_nums]
    bufferOffset = _serializer.uint32(obj.frontier_cell_nums.length, buffer, bufferOffset);
    obj.frontier_cell_nums.forEach((val) => {
      bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [contained_frontier_ids]
    // Serialize the length for message field [contained_frontier_ids]
    bufferOffset = _serializer.uint32(obj.contained_frontier_ids.length, buffer, bufferOffset);
    obj.contained_frontier_ids.forEach((val) => {
      bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [is_updated]
    bufferOffset = _arraySerializer.bool(obj.is_updated, buffer, bufferOffset, null);
    // Serialize message field [need_divide]
    bufferOffset = _arraySerializer.bool(obj.need_divide, buffer, bufferOffset, null);
    // Serialize message field [active]
    bufferOffset = _arraySerializer.bool(obj.active, buffer, bufferOffset, null);
    // Serialize message field [is_prev_relevant]
    bufferOffset = _arraySerializer.bool(obj.is_prev_relevant, buffer, bufferOffset, null);
    // Serialize message field [is_cur_relevant]
    bufferOffset = _arraySerializer.bool(obj.is_cur_relevant, buffer, bufferOffset, null);
    // Serialize message field [is_covered]
    bufferOffset = _arraySerializer.bool(obj.is_covered, buffer, bufferOffset, null);
    // Serialize message field [relevant_id]
    bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(obj.relevant_id, buffer, bufferOffset);
    // Serialize message field [relevant_map]
    bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(obj.relevant_map, buffer, bufferOffset);
    // Serialize message field [min]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.min, buffer, bufferOffset);
    // Serialize message field [max]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.max, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.resolution, buffer, bufferOffset);
    // Serialize message field [grid_size]
    bufferOffset = _serializer.float32(obj.grid_size, buffer, bufferOffset);
    // Serialize message field [multi_layer_hgrid]
    bufferOffset = _serializer.bool(obj.multi_layer_hgrid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HgridMsg
    let len;
    let data = new HgridMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [recv]
    data.recv = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [local_id]
    data.local_id = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [unknown_num]
    data.unknown_num = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [frontier_num]
    data.frontier_num = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [center]
    // Deserialize array length for message field [center]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.center = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.center[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [frontier_cell_nums]
    // Deserialize array length for message field [frontier_cell_nums]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.frontier_cell_nums = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.frontier_cell_nums[i] = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [contained_frontier_ids]
    // Deserialize array length for message field [contained_frontier_ids]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.contained_frontier_ids = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.contained_frontier_ids[i] = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [is_updated]
    data.is_updated = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [need_divide]
    data.need_divide = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [active]
    data.active = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [is_prev_relevant]
    data.is_prev_relevant = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [is_cur_relevant]
    data.is_cur_relevant = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [is_covered]
    data.is_covered = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [relevant_id]
    data.relevant_id = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [relevant_map]
    data.relevant_map = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [min]
    data.min = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [max]
    data.max = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [grid_size]
    data.grid_size = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [multi_layer_hgrid]
    data.multi_layer_hgrid = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 2 * object.id.length;
    length += 2 * object.local_id.length;
    length += 2 * object.unknown_num.length;
    length += 2 * object.frontier_num.length;
    length += 24 * object.center.length;
    object.frontier_cell_nums.forEach((val) => {
      length += std_msgs.msg.UInt16MultiArray.getMessageSize(val);
    });
    object.contained_frontier_ids.forEach((val) => {
      length += std_msgs.msg.UInt16MultiArray.getMessageSize(val);
    });
    length += object.is_updated.length;
    length += object.need_divide.length;
    length += object.active.length;
    length += object.is_prev_relevant.length;
    length += object.is_cur_relevant.length;
    length += object.is_covered.length;
    length += std_msgs.msg.UInt16MultiArray.getMessageSize(object.relevant_id);
    length += std_msgs.msg.UInt16MultiArray.getMessageSize(object.relevant_map);
    return length + 130;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/HgridMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd287d82d0713c1a695d6c48be1e56cc3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HgridMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.recv !== undefined) {
      resolved.recv = msg.recv;
    }
    else {
      resolved.recv = false
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = []
    }

    if (msg.local_id !== undefined) {
      resolved.local_id = msg.local_id;
    }
    else {
      resolved.local_id = []
    }

    if (msg.unknown_num !== undefined) {
      resolved.unknown_num = msg.unknown_num;
    }
    else {
      resolved.unknown_num = []
    }

    if (msg.frontier_num !== undefined) {
      resolved.frontier_num = msg.frontier_num;
    }
    else {
      resolved.frontier_num = []
    }

    if (msg.center !== undefined) {
      resolved.center = new Array(msg.center.length);
      for (let i = 0; i < resolved.center.length; ++i) {
        resolved.center[i] = geometry_msgs.msg.Point.Resolve(msg.center[i]);
      }
    }
    else {
      resolved.center = []
    }

    if (msg.frontier_cell_nums !== undefined) {
      resolved.frontier_cell_nums = new Array(msg.frontier_cell_nums.length);
      for (let i = 0; i < resolved.frontier_cell_nums.length; ++i) {
        resolved.frontier_cell_nums[i] = std_msgs.msg.UInt16MultiArray.Resolve(msg.frontier_cell_nums[i]);
      }
    }
    else {
      resolved.frontier_cell_nums = []
    }

    if (msg.contained_frontier_ids !== undefined) {
      resolved.contained_frontier_ids = new Array(msg.contained_frontier_ids.length);
      for (let i = 0; i < resolved.contained_frontier_ids.length; ++i) {
        resolved.contained_frontier_ids[i] = std_msgs.msg.UInt16MultiArray.Resolve(msg.contained_frontier_ids[i]);
      }
    }
    else {
      resolved.contained_frontier_ids = []
    }

    if (msg.is_updated !== undefined) {
      resolved.is_updated = msg.is_updated;
    }
    else {
      resolved.is_updated = []
    }

    if (msg.need_divide !== undefined) {
      resolved.need_divide = msg.need_divide;
    }
    else {
      resolved.need_divide = []
    }

    if (msg.active !== undefined) {
      resolved.active = msg.active;
    }
    else {
      resolved.active = []
    }

    if (msg.is_prev_relevant !== undefined) {
      resolved.is_prev_relevant = msg.is_prev_relevant;
    }
    else {
      resolved.is_prev_relevant = []
    }

    if (msg.is_cur_relevant !== undefined) {
      resolved.is_cur_relevant = msg.is_cur_relevant;
    }
    else {
      resolved.is_cur_relevant = []
    }

    if (msg.is_covered !== undefined) {
      resolved.is_covered = msg.is_covered;
    }
    else {
      resolved.is_covered = []
    }

    if (msg.relevant_id !== undefined) {
      resolved.relevant_id = std_msgs.msg.UInt16MultiArray.Resolve(msg.relevant_id)
    }
    else {
      resolved.relevant_id = new std_msgs.msg.UInt16MultiArray()
    }

    if (msg.relevant_map !== undefined) {
      resolved.relevant_map = std_msgs.msg.UInt16MultiArray.Resolve(msg.relevant_map)
    }
    else {
      resolved.relevant_map = new std_msgs.msg.UInt16MultiArray()
    }

    if (msg.min !== undefined) {
      resolved.min = geometry_msgs.msg.Point.Resolve(msg.min)
    }
    else {
      resolved.min = new geometry_msgs.msg.Point()
    }

    if (msg.max !== undefined) {
      resolved.max = geometry_msgs.msg.Point.Resolve(msg.max)
    }
    else {
      resolved.max = new geometry_msgs.msg.Point()
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = geometry_msgs.msg.Point.Resolve(msg.resolution)
    }
    else {
      resolved.resolution = new geometry_msgs.msg.Point()
    }

    if (msg.grid_size !== undefined) {
      resolved.grid_size = msg.grid_size;
    }
    else {
      resolved.grid_size = 0.0
    }

    if (msg.multi_layer_hgrid !== undefined) {
      resolved.multi_layer_hgrid = msg.multi_layer_hgrid;
    }
    else {
      resolved.multi_layer_hgrid = false
    }

    return resolved;
    }
};

module.exports = HgridMsg;
