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

class FtrPointArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.PointArray = null;
    }
    else {
      if (initObj.hasOwnProperty('PointArray')) {
        this.PointArray = initObj.PointArray
      }
      else {
        this.PointArray = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FtrPointArray
    // Serialize message field [PointArray]
    // Serialize the length for message field [PointArray]
    bufferOffset = _serializer.uint32(obj.PointArray.length, buffer, bufferOffset);
    obj.PointArray.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FtrPointArray
    let len;
    let data = new FtrPointArray(null);
    // Deserialize message field [PointArray]
    // Deserialize array length for message field [PointArray]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.PointArray = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.PointArray[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.PointArray.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/FtrPointArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b72da7cba93373ecc2fd54ff85f47989';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new FtrPointArray(null);
    if (msg.PointArray !== undefined) {
      resolved.PointArray = new Array(msg.PointArray.length);
      for (let i = 0; i < resolved.PointArray.length; ++i) {
        resolved.PointArray[i] = geometry_msgs.msg.Point.Resolve(msg.PointArray[i]);
      }
    }
    else {
      resolved.PointArray = []
    }

    return resolved;
    }
};

module.exports = FtrPointArray;
