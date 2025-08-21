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

class FormationId {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_formation_id = null;
      this.car_formation_id = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_formation_id')) {
        this.drone_formation_id = initObj.drone_formation_id
      }
      else {
        this.drone_formation_id = 0;
      }
      if (initObj.hasOwnProperty('car_formation_id')) {
        this.car_formation_id = initObj.car_formation_id
      }
      else {
        this.car_formation_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FormationId
    // Serialize message field [drone_formation_id]
    bufferOffset = _serializer.int16(obj.drone_formation_id, buffer, bufferOffset);
    // Serialize message field [car_formation_id]
    bufferOffset = _serializer.int16(obj.car_formation_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FormationId
    let len;
    let data = new FormationId(null);
    // Deserialize message field [drone_formation_id]
    data.drone_formation_id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [car_formation_id]
    data.car_formation_id = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/FormationId';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b546e1d2050d5e3211b5e5e45523d88e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 drone_formation_id
    int16 car_formation_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FormationId(null);
    if (msg.drone_formation_id !== undefined) {
      resolved.drone_formation_id = msg.drone_formation_id;
    }
    else {
      resolved.drone_formation_id = 0
    }

    if (msg.car_formation_id !== undefined) {
      resolved.car_formation_id = msg.car_formation_id;
    }
    else {
      resolved.car_formation_id = 0
    }

    return resolved;
    }
};

module.exports = FormationId;
