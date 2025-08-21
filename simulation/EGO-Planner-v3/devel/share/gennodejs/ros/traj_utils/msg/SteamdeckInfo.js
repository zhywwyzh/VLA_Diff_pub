// Auto-generated. Do not edit!

// (in-package traj_utils.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FormationId = require('./FormationId.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class SteamdeckInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.traj_start_trigger = null;
      this.formation_ids = null;
    }
    else {
      if (initObj.hasOwnProperty('traj_start_trigger')) {
        this.traj_start_trigger = initObj.traj_start_trigger
      }
      else {
        this.traj_start_trigger = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('formation_ids')) {
        this.formation_ids = initObj.formation_ids
      }
      else {
        this.formation_ids = new FormationId();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SteamdeckInfo
    // Serialize message field [traj_start_trigger]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.traj_start_trigger, buffer, bufferOffset);
    // Serialize message field [formation_ids]
    bufferOffset = FormationId.serialize(obj.formation_ids, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SteamdeckInfo
    let len;
    let data = new SteamdeckInfo(null);
    // Deserialize message field [traj_start_trigger]
    data.traj_start_trigger = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [formation_ids]
    data.formation_ids = FormationId.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.traj_start_trigger);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_utils/SteamdeckInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ed9ffc74ffa217ce2263aeb795f0421';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # command_realted
    geometry_msgs/PoseStamped traj_start_trigger
    traj_utils/FormationId formation_ids
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: traj_utils/FormationId
    int16 drone_formation_id
    int16 car_formation_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SteamdeckInfo(null);
    if (msg.traj_start_trigger !== undefined) {
      resolved.traj_start_trigger = geometry_msgs.msg.PoseStamped.Resolve(msg.traj_start_trigger)
    }
    else {
      resolved.traj_start_trigger = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.formation_ids !== undefined) {
      resolved.formation_ids = FormationId.Resolve(msg.formation_ids)
    }
    else {
      resolved.formation_ids = new FormationId()
    }

    return resolved;
    }
};

module.exports = SteamdeckInfo;
