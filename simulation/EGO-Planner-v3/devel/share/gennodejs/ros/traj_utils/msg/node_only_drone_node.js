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

class node_only_drone_node {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.node_already_reastart = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('node_already_reastart')) {
        this.node_already_reastart = initObj.node_already_reastart
      }
      else {
        this.node_already_reastart = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type node_only_drone_node
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int8(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [node_already_reastart]
    bufferOffset = _serializer.int8(obj.node_already_reastart, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type node_only_drone_node
    let len;
    let data = new node_only_drone_node(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [node_already_reastart]
    data.node_already_reastart = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/node_only_drone_node';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33299fffa442f68d79e075c04bbd062c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 drone_id
    int8 node_already_reastart
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new node_only_drone_node(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.node_already_reastart !== undefined) {
      resolved.node_already_reastart = msg.node_already_reastart;
    }
    else {
      resolved.node_already_reastart = 0
    }

    return resolved;
    }
};

module.exports = node_only_drone_node;
