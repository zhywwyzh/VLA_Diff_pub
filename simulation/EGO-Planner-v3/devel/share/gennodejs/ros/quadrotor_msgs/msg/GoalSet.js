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

//-----------------------------------------------------------

class GoalSet {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.to_drone_ids = null;
      this.goal = null;
      this.yaw = null;
      this.look_forward = null;
      this.goal_to_follower = null;
    }
    else {
      if (initObj.hasOwnProperty('to_drone_ids')) {
        this.to_drone_ids = initObj.to_drone_ids
      }
      else {
        this.to_drone_ids = [];
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = [];
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = [];
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
    // Serializes a message object of type GoalSet
    // Serialize message field [to_drone_ids]
    bufferOffset = _arraySerializer.uint8(obj.to_drone_ids, buffer, bufferOffset, null);
    // Serialize message field [goal]
    // Serialize the length for message field [goal]
    bufferOffset = _serializer.uint32(obj.goal.length, buffer, bufferOffset);
    obj.goal.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [yaw]
    bufferOffset = _arraySerializer.float32(obj.yaw, buffer, bufferOffset, null);
    // Serialize message field [look_forward]
    bufferOffset = _serializer.bool(obj.look_forward, buffer, bufferOffset);
    // Serialize message field [goal_to_follower]
    bufferOffset = _serializer.bool(obj.goal_to_follower, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoalSet
    let len;
    let data = new GoalSet(null);
    // Deserialize message field [to_drone_ids]
    data.to_drone_ids = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [goal]
    // Deserialize array length for message field [goal]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.goal = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.goal[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [yaw]
    data.yaw = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [look_forward]
    data.look_forward = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [goal_to_follower]
    data.goal_to_follower = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.to_drone_ids.length;
    length += 24 * object.goal.length;
    length += 4 * object.yaw.length;
    return length + 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/GoalSet';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fce849fd3f9e593aecb0eb34b6f685a3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[]               to_drone_ids
    geometry_msgs/Point[] goal
    float32[]             yaw
    bool                  look_forward
    bool                  goal_to_follower
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
    const resolved = new GoalSet(null);
    if (msg.to_drone_ids !== undefined) {
      resolved.to_drone_ids = msg.to_drone_ids;
    }
    else {
      resolved.to_drone_ids = []
    }

    if (msg.goal !== undefined) {
      resolved.goal = new Array(msg.goal.length);
      for (let i = 0; i < resolved.goal.length; ++i) {
        resolved.goal[i] = geometry_msgs.msg.Point.Resolve(msg.goal[i]);
      }
    }
    else {
      resolved.goal = []
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = []
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

module.exports = GoalSet;
