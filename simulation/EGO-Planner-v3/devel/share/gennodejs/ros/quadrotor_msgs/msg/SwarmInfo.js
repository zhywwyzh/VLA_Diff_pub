// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SwarmInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.swarm_id = null;
      this.robot_ids = null;
      this.leader_id = null;
    }
    else {
      if (initObj.hasOwnProperty('swarm_id')) {
        this.swarm_id = initObj.swarm_id
      }
      else {
        this.swarm_id = 0;
      }
      if (initObj.hasOwnProperty('robot_ids')) {
        this.robot_ids = initObj.robot_ids
      }
      else {
        this.robot_ids = [];
      }
      if (initObj.hasOwnProperty('leader_id')) {
        this.leader_id = initObj.leader_id
      }
      else {
        this.leader_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwarmInfo
    // Serialize message field [swarm_id]
    bufferOffset = _serializer.int32(obj.swarm_id, buffer, bufferOffset);
    // Serialize message field [robot_ids]
    bufferOffset = _arraySerializer.int32(obj.robot_ids, buffer, bufferOffset, null);
    // Serialize message field [leader_id]
    bufferOffset = _serializer.int32(obj.leader_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwarmInfo
    let len;
    let data = new SwarmInfo(null);
    // Deserialize message field [swarm_id]
    data.swarm_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [robot_ids]
    data.robot_ids = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [leader_id]
    data.leader_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.robot_ids.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/SwarmInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e58f24709b97dba610bf92bfa971d6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 swarm_id
    int32[] robot_ids
    int32 leader_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SwarmInfo(null);
    if (msg.swarm_id !== undefined) {
      resolved.swarm_id = msg.swarm_id;
    }
    else {
      resolved.swarm_id = 0
    }

    if (msg.robot_ids !== undefined) {
      resolved.robot_ids = msg.robot_ids;
    }
    else {
      resolved.robot_ids = []
    }

    if (msg.leader_id !== undefined) {
      resolved.leader_id = msg.leader_id;
    }
    else {
      resolved.leader_id = 0
    }

    return resolved;
    }
};

module.exports = SwarmInfo;
