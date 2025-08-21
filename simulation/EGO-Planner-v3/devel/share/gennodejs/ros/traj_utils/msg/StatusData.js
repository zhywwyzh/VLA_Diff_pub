// Auto-generated. Do not edit!

// (in-package traj_utils.msg)


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

class StatusData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drone_id = null;
      this.loop_rate = null;
      this.voltage = null;
      this.seq = null;
      this.dead = null;
      this.dead_pos = null;
      this.dead_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('loop_rate')) {
        this.loop_rate = initObj.loop_rate
      }
      else {
        this.loop_rate = 0;
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('seq')) {
        this.seq = initObj.seq
      }
      else {
        this.seq = 0;
      }
      if (initObj.hasOwnProperty('dead')) {
        this.dead = initObj.dead
      }
      else {
        this.dead = 0;
      }
      if (initObj.hasOwnProperty('dead_pos')) {
        this.dead_pos = initObj.dead_pos
      }
      else {
        this.dead_pos = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('dead_vel')) {
        this.dead_vel = initObj.dead_vel
      }
      else {
        this.dead_vel = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StatusData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drone_id]
    bufferOffset = _serializer.uint8(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [loop_rate]
    bufferOffset = _serializer.uint16(obj.loop_rate, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float64(obj.voltage, buffer, bufferOffset);
    // Serialize message field [seq]
    bufferOffset = _serializer.uint8(obj.seq, buffer, bufferOffset);
    // Serialize message field [dead]
    bufferOffset = _serializer.uint8(obj.dead, buffer, bufferOffset);
    // Serialize message field [dead_pos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.dead_pos, buffer, bufferOffset);
    // Serialize message field [dead_vel]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.dead_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StatusData
    let len;
    let data = new StatusData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [loop_rate]
    data.loop_rate = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [seq]
    data.seq = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dead]
    data.dead = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dead_pos]
    data.dead_pos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [dead_vel]
    data.dead_vel = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 61;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/StatusData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fb7b2c8a913bcd9ee73aff94ba73e8f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint8 drone_id
    uint16 loop_rate
    float64 voltage
    uint8 seq
    uint8 dead
    geometry_msgs/Point dead_pos
    geometry_msgs/Point dead_vel
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StatusData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.loop_rate !== undefined) {
      resolved.loop_rate = msg.loop_rate;
    }
    else {
      resolved.loop_rate = 0
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.seq !== undefined) {
      resolved.seq = msg.seq;
    }
    else {
      resolved.seq = 0
    }

    if (msg.dead !== undefined) {
      resolved.dead = msg.dead;
    }
    else {
      resolved.dead = 0
    }

    if (msg.dead_pos !== undefined) {
      resolved.dead_pos = geometry_msgs.msg.Point.Resolve(msg.dead_pos)
    }
    else {
      resolved.dead_pos = new geometry_msgs.msg.Point()
    }

    if (msg.dead_vel !== undefined) {
      resolved.dead_vel = geometry_msgs.msg.Point.Resolve(msg.dead_vel)
    }
    else {
      resolved.dead_vel = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = StatusData;
