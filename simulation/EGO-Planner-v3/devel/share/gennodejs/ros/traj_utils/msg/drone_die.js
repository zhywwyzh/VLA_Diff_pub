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

class drone_die {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_die = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_die')) {
        this.drone_die = initObj.drone_die
      }
      else {
        this.drone_die = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drone_die
    // Serialize message field [drone_die]
    bufferOffset = _serializer.int32(obj.drone_die, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drone_die
    let len;
    let data = new drone_die(null);
    // Deserialize message field [drone_die]
    data.drone_die = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/drone_die';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '469265ba708544468cfac3f514285ad9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 drone_die
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drone_die(null);
    if (msg.drone_die !== undefined) {
      resolved.drone_die = msg.drone_die;
    }
    else {
      resolved.drone_die = 0
    }

    return resolved;
    }
};

module.exports = drone_die;
