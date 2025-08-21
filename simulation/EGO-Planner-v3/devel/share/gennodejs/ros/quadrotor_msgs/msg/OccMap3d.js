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

//-----------------------------------------------------------

class OccMap3d {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.resolution = null;
      this.inflate_size = null;
      this.size_x = null;
      this.size_y = null;
      this.size_z = null;
      this.offset_x = null;
      this.offset_y = null;
      this.offset_z = null;
      this.data = null;
      this.esdf = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0.0;
      }
      if (initObj.hasOwnProperty('inflate_size')) {
        this.inflate_size = initObj.inflate_size
      }
      else {
        this.inflate_size = 0;
      }
      if (initObj.hasOwnProperty('size_x')) {
        this.size_x = initObj.size_x
      }
      else {
        this.size_x = 0;
      }
      if (initObj.hasOwnProperty('size_y')) {
        this.size_y = initObj.size_y
      }
      else {
        this.size_y = 0;
      }
      if (initObj.hasOwnProperty('size_z')) {
        this.size_z = initObj.size_z
      }
      else {
        this.size_z = 0;
      }
      if (initObj.hasOwnProperty('offset_x')) {
        this.offset_x = initObj.offset_x
      }
      else {
        this.offset_x = 0;
      }
      if (initObj.hasOwnProperty('offset_y')) {
        this.offset_y = initObj.offset_y
      }
      else {
        this.offset_y = 0;
      }
      if (initObj.hasOwnProperty('offset_z')) {
        this.offset_z = initObj.offset_z
      }
      else {
        this.offset_z = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
      if (initObj.hasOwnProperty('esdf')) {
        this.esdf = initObj.esdf
      }
      else {
        this.esdf = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OccMap3d
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.float32(obj.resolution, buffer, bufferOffset);
    // Serialize message field [inflate_size]
    bufferOffset = _serializer.int16(obj.inflate_size, buffer, bufferOffset);
    // Serialize message field [size_x]
    bufferOffset = _serializer.int16(obj.size_x, buffer, bufferOffset);
    // Serialize message field [size_y]
    bufferOffset = _serializer.int16(obj.size_y, buffer, bufferOffset);
    // Serialize message field [size_z]
    bufferOffset = _serializer.int16(obj.size_z, buffer, bufferOffset);
    // Serialize message field [offset_x]
    bufferOffset = _serializer.int16(obj.offset_x, buffer, bufferOffset);
    // Serialize message field [offset_y]
    bufferOffset = _serializer.int16(obj.offset_y, buffer, bufferOffset);
    // Serialize message field [offset_z]
    bufferOffset = _serializer.int16(obj.offset_z, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.int8(obj.data, buffer, bufferOffset, null);
    // Serialize message field [esdf]
    bufferOffset = _arraySerializer.float64(obj.esdf, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OccMap3d
    let len;
    let data = new OccMap3d(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [inflate_size]
    data.inflate_size = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [size_x]
    data.size_x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [size_y]
    data.size_y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [size_z]
    data.size_z = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [offset_x]
    data.offset_x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [offset_y]
    data.offset_y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [offset_z]
    data.offset_z = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [esdf]
    data.esdf = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.data.length;
    length += 8 * object.esdf.length;
    return length + 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/OccMap3d';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9fb1de7c912d694532f4f1aa26096e06';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float32 resolution
    int16 inflate_size
    int16 size_x
    int16 size_y
    int16 size_z
    int16 offset_x
    int16 offset_y
    int16 offset_z
    
    int8[] data
    float64[] esdf
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OccMap3d(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0.0
    }

    if (msg.inflate_size !== undefined) {
      resolved.inflate_size = msg.inflate_size;
    }
    else {
      resolved.inflate_size = 0
    }

    if (msg.size_x !== undefined) {
      resolved.size_x = msg.size_x;
    }
    else {
      resolved.size_x = 0
    }

    if (msg.size_y !== undefined) {
      resolved.size_y = msg.size_y;
    }
    else {
      resolved.size_y = 0
    }

    if (msg.size_z !== undefined) {
      resolved.size_z = msg.size_z;
    }
    else {
      resolved.size_z = 0
    }

    if (msg.offset_x !== undefined) {
      resolved.offset_x = msg.offset_x;
    }
    else {
      resolved.offset_x = 0
    }

    if (msg.offset_y !== undefined) {
      resolved.offset_y = msg.offset_y;
    }
    else {
      resolved.offset_y = 0
    }

    if (msg.offset_z !== undefined) {
      resolved.offset_z = msg.offset_z;
    }
    else {
      resolved.offset_z = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    if (msg.esdf !== undefined) {
      resolved.esdf = msg.esdf;
    }
    else {
      resolved.esdf = []
    }

    return resolved;
    }
};

module.exports = OccMap3d;
