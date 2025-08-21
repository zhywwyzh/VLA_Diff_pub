// Auto-generated. Do not edit!

// (in-package quadrotor_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LinktrackNodeframe3 = require('./LinktrackNodeframe3.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DistanceMeas {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.distance_meas = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('distance_meas')) {
        this.distance_meas = initObj.distance_meas
      }
      else {
        this.distance_meas = new LinktrackNodeframe3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DistanceMeas
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [distance_meas]
    bufferOffset = LinktrackNodeframe3.serialize(obj.distance_meas, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DistanceMeas
    let len;
    let data = new DistanceMeas(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [distance_meas]
    data.distance_meas = LinktrackNodeframe3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += LinktrackNodeframe3.getMessageSize(object.distance_meas);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/DistanceMeas';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c3b6590cda39f0c3a802590fc69840f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    quadrotor_msgs/LinktrackNodeframe3 distance_meas
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
    
    ================================================================================
    MSG: quadrotor_msgs/LinktrackNodeframe3
    uint8 role
    uint8 id
    uint32 local_time
    uint32 system_time
    float32 voltage
    LinktrackNode2[] nodes
    
    ================================================================================
    MSG: quadrotor_msgs/LinktrackNode2
    uint8 role
    uint8 id
    float32 dis
    float32 fp_rssi
    float32 rx_rssi
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DistanceMeas(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.distance_meas !== undefined) {
      resolved.distance_meas = LinktrackNodeframe3.Resolve(msg.distance_meas)
    }
    else {
      resolved.distance_meas = new LinktrackNodeframe3()
    }

    return resolved;
    }
};

module.exports = DistanceMeas;
