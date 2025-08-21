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

class WayPointInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.waypoint_id = null;
      this.waypoint_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('waypoint_id')) {
        this.waypoint_id = initObj.waypoint_id
      }
      else {
        this.waypoint_id = 0;
      }
      if (initObj.hasOwnProperty('waypoint_pos')) {
        this.waypoint_pos = initObj.waypoint_pos
      }
      else {
        this.waypoint_pos = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WayPointInfo
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int16(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [waypoint_id]
    bufferOffset = _serializer.int16(obj.waypoint_id, buffer, bufferOffset);
    // Check that the constant length array field [waypoint_pos] has the right length
    if (obj.waypoint_pos.length !== 3) {
      throw new Error('Unable to serialize array field waypoint_pos - length must be 3')
    }
    // Serialize message field [waypoint_pos]
    bufferOffset = _arraySerializer.float32(obj.waypoint_pos, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WayPointInfo
    let len;
    let data = new WayPointInfo(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [waypoint_id]
    data.waypoint_id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [waypoint_pos]
    data.waypoint_pos = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/WayPointInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f3e1a39e25f76f15df8b36b8cc49d0af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 drone_id
    int16 waypoint_id
    float32[3] waypoint_pos
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WayPointInfo(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.waypoint_id !== undefined) {
      resolved.waypoint_id = msg.waypoint_id;
    }
    else {
      resolved.waypoint_id = 0
    }

    if (msg.waypoint_pos !== undefined) {
      resolved.waypoint_pos = msg.waypoint_pos;
    }
    else {
      resolved.waypoint_pos = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = WayPointInfo;
