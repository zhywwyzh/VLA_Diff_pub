// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MultiPoseGraph {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.key_pose_list_xyz = null;
      this.key_pose_list_intensity = null;
      this.pose_edge_p_end = null;
      this.pose_edge_weight = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('key_pose_list_xyz')) {
        this.key_pose_list_xyz = initObj.key_pose_list_xyz
      }
      else {
        this.key_pose_list_xyz = [];
      }
      if (initObj.hasOwnProperty('key_pose_list_intensity')) {
        this.key_pose_list_intensity = initObj.key_pose_list_intensity
      }
      else {
        this.key_pose_list_intensity = [];
      }
      if (initObj.hasOwnProperty('pose_edge_p_end')) {
        this.pose_edge_p_end = initObj.pose_edge_p_end
      }
      else {
        this.pose_edge_p_end = [];
      }
      if (initObj.hasOwnProperty('pose_edge_weight')) {
        this.pose_edge_weight = initObj.pose_edge_weight
      }
      else {
        this.pose_edge_weight = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MultiPoseGraph
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [key_pose_list_xyz]
    // Serialize the length for message field [key_pose_list_xyz]
    bufferOffset = _serializer.uint32(obj.key_pose_list_xyz.length, buffer, bufferOffset);
    obj.key_pose_list_xyz.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [key_pose_list_intensity]
    bufferOffset = _arraySerializer.float32(obj.key_pose_list_intensity, buffer, bufferOffset, null);
    // Serialize message field [pose_edge_p_end]
    // Serialize the length for message field [pose_edge_p_end]
    bufferOffset = _serializer.uint32(obj.pose_edge_p_end.length, buffer, bufferOffset);
    obj.pose_edge_p_end.forEach((val) => {
      bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [pose_edge_weight]
    // Serialize the length for message field [pose_edge_weight]
    bufferOffset = _serializer.uint32(obj.pose_edge_weight.length, buffer, bufferOffset);
    obj.pose_edge_weight.forEach((val) => {
      bufferOffset = std_msgs.msg.Float32MultiArray.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MultiPoseGraph
    let len;
    let data = new MultiPoseGraph(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [key_pose_list_xyz]
    // Deserialize array length for message field [key_pose_list_xyz]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.key_pose_list_xyz = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.key_pose_list_xyz[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [key_pose_list_intensity]
    data.key_pose_list_intensity = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [pose_edge_p_end]
    // Deserialize array length for message field [pose_edge_p_end]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pose_edge_p_end = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pose_edge_p_end[i] = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [pose_edge_weight]
    // Deserialize array length for message field [pose_edge_weight]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pose_edge_weight = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pose_edge_weight[i] = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.key_pose_list_xyz.length;
    length += 4 * object.key_pose_list_intensity.length;
    object.pose_edge_p_end.forEach((val) => {
      length += std_msgs.msg.UInt16MultiArray.getMessageSize(val);
    });
    object.pose_edge_weight.forEach((val) => {
      length += std_msgs.msg.Float32MultiArray.getMessageSize(val);
    });
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/MultiPoseGraph';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1253e98cd3689b7af6c77b6941e0ebdc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MultiPoseGraph(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.key_pose_list_xyz !== undefined) {
      resolved.key_pose_list_xyz = new Array(msg.key_pose_list_xyz.length);
      for (let i = 0; i < resolved.key_pose_list_xyz.length; ++i) {
        resolved.key_pose_list_xyz[i] = geometry_msgs.msg.Point.Resolve(msg.key_pose_list_xyz[i]);
      }
    }
    else {
      resolved.key_pose_list_xyz = []
    }

    if (msg.key_pose_list_intensity !== undefined) {
      resolved.key_pose_list_intensity = msg.key_pose_list_intensity;
    }
    else {
      resolved.key_pose_list_intensity = []
    }

    if (msg.pose_edge_p_end !== undefined) {
      resolved.pose_edge_p_end = new Array(msg.pose_edge_p_end.length);
      for (let i = 0; i < resolved.pose_edge_p_end.length; ++i) {
        resolved.pose_edge_p_end[i] = std_msgs.msg.UInt16MultiArray.Resolve(msg.pose_edge_p_end[i]);
      }
    }
    else {
      resolved.pose_edge_p_end = []
    }

    if (msg.pose_edge_weight !== undefined) {
      resolved.pose_edge_weight = new Array(msg.pose_edge_weight.length);
      for (let i = 0; i < resolved.pose_edge_weight.length; ++i) {
        resolved.pose_edge_weight[i] = std_msgs.msg.Float32MultiArray.Resolve(msg.pose_edge_weight[i]);
      }
    }
    else {
      resolved.pose_edge_weight = []
    }

    return resolved;
    }
};

module.exports = MultiPoseGraph;
