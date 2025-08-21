// Auto-generated. Do not edit!

// (in-package traj_utils.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CarTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_stamp = null;
      this.dt = null;
      this.X = null;
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('start_stamp')) {
        this.start_stamp = initObj.start_stamp
      }
      else {
        this.start_stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('dt')) {
        this.dt = initObj.dt
      }
      else {
        this.dt = 0.0;
      }
      if (initObj.hasOwnProperty('X')) {
        this.X = initObj.X
      }
      else {
        this.X = [];
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CarTraj
    // Serialize message field [start_stamp]
    bufferOffset = _serializer.time(obj.start_stamp, buffer, bufferOffset);
    // Serialize message field [dt]
    bufferOffset = _serializer.float64(obj.dt, buffer, bufferOffset);
    // Serialize message field [X]
    bufferOffset = _arraySerializer.float64(obj.X, buffer, bufferOffset, null);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CarTraj
    let len;
    let data = new CarTraj(null);
    // Deserialize message field [start_stamp]
    data.start_stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [dt]
    data.dt = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [X]
    data.X = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.X.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/CarTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '951e389eb9cdbc57941969e38b64973a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time start_stamp
    float64 dt
    float64[] X
    int32 id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CarTraj(null);
    if (msg.start_stamp !== undefined) {
      resolved.start_stamp = msg.start_stamp;
    }
    else {
      resolved.start_stamp = {secs: 0, nsecs: 0}
    }

    if (msg.dt !== undefined) {
      resolved.dt = msg.dt;
    }
    else {
      resolved.dt = 0.0
    }

    if (msg.X !== undefined) {
      resolved.X = msg.X;
    }
    else {
      resolved.X = []
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

module.exports = CarTraj;
