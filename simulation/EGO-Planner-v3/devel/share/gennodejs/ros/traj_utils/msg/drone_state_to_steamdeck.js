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

class drone_state_to_steamdeck {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.start_time = null;
      this.current_node_state = null;
      this.debug_info = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('current_node_state')) {
        this.current_node_state = initObj.current_node_state
      }
      else {
        this.current_node_state = 0;
      }
      if (initObj.hasOwnProperty('debug_info')) {
        this.debug_info = initObj.debug_info
      }
      else {
        this.debug_info = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drone_state_to_steamdeck
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int8(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [current_node_state]
    bufferOffset = _serializer.int8(obj.current_node_state, buffer, bufferOffset);
    // Serialize message field [debug_info]
    bufferOffset = _serializer.string(obj.debug_info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drone_state_to_steamdeck
    let len;
    let data = new drone_state_to_steamdeck(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [current_node_state]
    data.current_node_state = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [debug_info]
    data.debug_info = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.debug_info);
    return length + 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/drone_state_to_steamdeck';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd799a4895f8a152d4ada9c7313ecec9b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 drone_id
    # 1: baseFail (vins_odom too low or no odom)
    # 2: basepreparing
    # 3: baseReady
    # 4: px4_take_off
    # 5: command
    # 6: px4_land
    # 7: auto_hover
    # 8: locatizationError (optical_flow)
    # 9: crash (cmd ---> stopPropeller) 
    # 10: drone_node_only
    # 11: timeAlignFail
    time start_time
    int8 current_node_state
    string debug_info
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drone_state_to_steamdeck(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.current_node_state !== undefined) {
      resolved.current_node_state = msg.current_node_state;
    }
    else {
      resolved.current_node_state = 0
    }

    if (msg.debug_info !== undefined) {
      resolved.debug_info = msg.debug_info;
    }
    else {
      resolved.debug_info = ''
    }

    return resolved;
    }
};

module.exports = drone_state_to_steamdeck;
