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

class EgoGoalSet {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.goal = null;
      this.yaw = null;
      this.look_forward = null;
      this.goal_to_follower = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('look_forward')) {
        this.look_forward = initObj.look_forward
      }
      else {
        this.look_forward = false;
      }
      if (initObj.hasOwnProperty('goal_to_follower')) {
        this.goal_to_follower = initObj.goal_to_follower
      }
      else {
        this.goal_to_follower = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EgoGoalSet
    // Serialize message field [drone_id]
    bufferOffset = _serializer.uint8(obj.drone_id, buffer, bufferOffset);
    // Check that the constant length array field [goal] has the right length
    if (obj.goal.length !== 3) {
      throw new Error('Unable to serialize array field goal - length must be 3')
    }
    // Serialize message field [goal]
    bufferOffset = _arraySerializer.float32(obj.goal, buffer, bufferOffset, 3);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [look_forward]
    bufferOffset = _serializer.bool(obj.look_forward, buffer, bufferOffset);
    // Serialize message field [goal_to_follower]
    bufferOffset = _serializer.bool(obj.goal_to_follower, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EgoGoalSet
    let len;
    let data = new EgoGoalSet(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [look_forward]
    data.look_forward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [goal_to_follower]
    data.goal_to_follower = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/EgoGoalSet';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e000e06493c05ae8165574a33ffc993';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8      drone_id
    float32[3] goal
    float32    yaw
    bool       look_forward
    bool       goal_to_follower
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EgoGoalSet(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.goal !== undefined) {
      resolved.goal = msg.goal;
    }
    else {
      resolved.goal = new Array(3).fill(0)
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.look_forward !== undefined) {
      resolved.look_forward = msg.look_forward;
    }
    else {
      resolved.look_forward = false
    }

    if (msg.goal_to_follower !== undefined) {
      resolved.goal_to_follower = msg.goal_to_follower;
    }
    else {
      resolved.goal_to_follower = false
    }

    return resolved;
    }
};

module.exports = EgoGoalSet;
