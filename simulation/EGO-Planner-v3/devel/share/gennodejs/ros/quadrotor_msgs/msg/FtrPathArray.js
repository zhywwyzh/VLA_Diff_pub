// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FtrPointArray = require('./FtrPointArray.js');

//-----------------------------------------------------------

class FtrPathArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.PathArray = null;
    }
    else {
      if (initObj.hasOwnProperty('PathArray')) {
        this.PathArray = initObj.PathArray
      }
      else {
        this.PathArray = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FtrPathArray
    // Serialize message field [PathArray]
    // Serialize the length for message field [PathArray]
    bufferOffset = _serializer.uint32(obj.PathArray.length, buffer, bufferOffset);
    obj.PathArray.forEach((val) => {
      bufferOffset = FtrPointArray.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FtrPathArray
    let len;
    let data = new FtrPathArray(null);
    // Deserialize message field [PathArray]
    // Deserialize array length for message field [PathArray]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.PathArray = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.PathArray[i] = FtrPointArray.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.PathArray.forEach((val) => {
      length += FtrPointArray.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/FtrPathArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd168064edf658ded54eac8a732d98b8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    FtrPointArray[] PathArray
    
    ================================================================================
    MSG: quadrotor_msgs/FtrPointArray
    geometry_msgs/Point[] PointArray
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
    const resolved = new FtrPathArray(null);
    if (msg.PathArray !== undefined) {
      resolved.PathArray = new Array(msg.PathArray.length);
      for (let i = 0; i < resolved.PathArray.length; ++i) {
        resolved.PathArray[i] = FtrPointArray.Resolve(msg.PathArray[i]);
      }
    }
    else {
      resolved.PathArray = []
    }

    return resolved;
    }
};

module.exports = FtrPathArray;
