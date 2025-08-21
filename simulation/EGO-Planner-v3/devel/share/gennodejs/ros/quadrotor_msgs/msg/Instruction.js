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

class Instruction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_id = null;
      this.instruction_type = null;
      this.target_position = null;
      this.src_drone_ids = null;
      this.tar_drone_ids = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = 0;
      }
      if (initObj.hasOwnProperty('instruction_type')) {
        this.instruction_type = initObj.instruction_type
      }
      else {
        this.instruction_type = 0;
      }
      if (initObj.hasOwnProperty('target_position')) {
        this.target_position = initObj.target_position
      }
      else {
        this.target_position = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('src_drone_ids')) {
        this.src_drone_ids = initObj.src_drone_ids
      }
      else {
        this.src_drone_ids = [];
      }
      if (initObj.hasOwnProperty('tar_drone_ids')) {
        this.tar_drone_ids = initObj.tar_drone_ids
      }
      else {
        this.tar_drone_ids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Instruction
    // Serialize message field [robot_id]
    bufferOffset = _serializer.uint8(obj.robot_id, buffer, bufferOffset);
    // Serialize message field [instruction_type]
    bufferOffset = _serializer.uint8(obj.instruction_type, buffer, bufferOffset);
    // Check that the constant length array field [target_position] has the right length
    if (obj.target_position.length !== 3) {
      throw new Error('Unable to serialize array field target_position - length must be 3')
    }
    // Serialize message field [target_position]
    bufferOffset = _arraySerializer.float32(obj.target_position, buffer, bufferOffset, 3);
    // Serialize message field [src_drone_ids]
    bufferOffset = _arraySerializer.uint16(obj.src_drone_ids, buffer, bufferOffset, null);
    // Serialize message field [tar_drone_ids]
    bufferOffset = _arraySerializer.uint16(obj.tar_drone_ids, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Instruction
    let len;
    let data = new Instruction(null);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [instruction_type]
    data.instruction_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [target_position]
    data.target_position = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [src_drone_ids]
    data.src_drone_ids = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [tar_drone_ids]
    data.tar_drone_ids = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.src_drone_ids.length;
    length += 2 * object.tar_drone_ids.length;
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/Instruction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6740b6e4e6245eb3348061673116e30f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 robot_id
    uint8 instruction_type
    float32[3] target_position
    
    # map merge request
    uint16[] src_drone_ids
    uint16[] tar_drone_ids
    
    #definations for instruction_type
    uint8 TURN_GOAL = 1
    uint8 TURN_EXPLORE = 2
    uint8 TURN_PATROL = 3
    uint8 TURN_HIT = 4
    uint8 GO_HOME = 5
    uint8 TURN_EGO_GOAL = 6
    uint8 SHARE_MAP = 7
    uint8 RESET_EXPLORE_AREA = 8
    uint8 MAP_MERGE_REQUEST = 9
    uint8 MAP_CIRCULATE = 10
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Instruction(null);
    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = 0
    }

    if (msg.instruction_type !== undefined) {
      resolved.instruction_type = msg.instruction_type;
    }
    else {
      resolved.instruction_type = 0
    }

    if (msg.target_position !== undefined) {
      resolved.target_position = msg.target_position;
    }
    else {
      resolved.target_position = new Array(3).fill(0)
    }

    if (msg.src_drone_ids !== undefined) {
      resolved.src_drone_ids = msg.src_drone_ids;
    }
    else {
      resolved.src_drone_ids = []
    }

    if (msg.tar_drone_ids !== undefined) {
      resolved.tar_drone_ids = msg.tar_drone_ids;
    }
    else {
      resolved.tar_drone_ids = []
    }

    return resolved;
    }
};

// Constants for message
Instruction.Constants = {
  TURN_GOAL: 1,
  TURN_EXPLORE: 2,
  TURN_PATROL: 3,
  TURN_HIT: 4,
  GO_HOME: 5,
  TURN_EGO_GOAL: 6,
  SHARE_MAP: 7,
  RESET_EXPLORE_AREA: 8,
  MAP_MERGE_REQUEST: 9,
  MAP_CIRCULATE: 10,
}

module.exports = Instruction;
