// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class InstructionResMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.instruction_type = null;
      this.drone_id = null;
      this.is_succeed = null;
      this.tar_drone_id = null;
      this.src_drone_id = null;
      this.failed_drone_id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('instruction_type')) {
        this.instruction_type = initObj.instruction_type
      }
      else {
        this.instruction_type = 0;
      }
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('is_succeed')) {
        this.is_succeed = initObj.is_succeed
      }
      else {
        this.is_succeed = false;
      }
      if (initObj.hasOwnProperty('tar_drone_id')) {
        this.tar_drone_id = initObj.tar_drone_id
      }
      else {
        this.tar_drone_id = [];
      }
      if (initObj.hasOwnProperty('src_drone_id')) {
        this.src_drone_id = initObj.src_drone_id
      }
      else {
        this.src_drone_id = [];
      }
      if (initObj.hasOwnProperty('failed_drone_id')) {
        this.failed_drone_id = initObj.failed_drone_id
      }
      else {
        this.failed_drone_id = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InstructionResMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [instruction_type]
    bufferOffset = _serializer.uint8(obj.instruction_type, buffer, bufferOffset);
    // Serialize message field [drone_id]
    bufferOffset = _serializer.uint8(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [is_succeed]
    bufferOffset = _serializer.bool(obj.is_succeed, buffer, bufferOffset);
    // Serialize message field [tar_drone_id]
    bufferOffset = _arraySerializer.uint8(obj.tar_drone_id, buffer, bufferOffset, null);
    // Serialize message field [src_drone_id]
    bufferOffset = _arraySerializer.uint8(obj.src_drone_id, buffer, bufferOffset, null);
    // Serialize message field [failed_drone_id]
    bufferOffset = _arraySerializer.uint8(obj.failed_drone_id, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InstructionResMsg
    let len;
    let data = new InstructionResMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [instruction_type]
    data.instruction_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [is_succeed]
    data.is_succeed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tar_drone_id]
    data.tar_drone_id = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [src_drone_id]
    data.src_drone_id = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [failed_drone_id]
    data.failed_drone_id = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.tar_drone_id.length;
    length += object.src_drone_id.length;
    length += object.failed_drone_id.length;
    return length + 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/InstructionResMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ebdd566ae6eb6d7e30fd1625d7dfda91';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint8 instruction_type
    uint8 drone_id
    bool is_succeed
    
    # -- map merge and share map -- #
    uint8[] tar_drone_id
    uint8[] src_drone_id
    uint8[] failed_drone_id
    
    # -- instruction type defination-- #
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InstructionResMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.instruction_type !== undefined) {
      resolved.instruction_type = msg.instruction_type;
    }
    else {
      resolved.instruction_type = 0
    }

    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.is_succeed !== undefined) {
      resolved.is_succeed = msg.is_succeed;
    }
    else {
      resolved.is_succeed = false
    }

    if (msg.tar_drone_id !== undefined) {
      resolved.tar_drone_id = msg.tar_drone_id;
    }
    else {
      resolved.tar_drone_id = []
    }

    if (msg.src_drone_id !== undefined) {
      resolved.src_drone_id = msg.src_drone_id;
    }
    else {
      resolved.src_drone_id = []
    }

    if (msg.failed_drone_id !== undefined) {
      resolved.failed_drone_id = msg.failed_drone_id;
    }
    else {
      resolved.failed_drone_id = []
    }

    return resolved;
    }
};

// Constants for message
InstructionResMsg.Constants = {
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

module.exports = InstructionResMsg;
