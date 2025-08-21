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

class EgoPlannerResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.planner_goal = null;
      this.plan_times = null;
      this.plan_status = null;
      this.modify_status = null;
    }
    else {
      if (initObj.hasOwnProperty('planner_goal')) {
        this.planner_goal = initObj.planner_goal
      }
      else {
        this.planner_goal = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('plan_times')) {
        this.plan_times = initObj.plan_times
      }
      else {
        this.plan_times = 0;
      }
      if (initObj.hasOwnProperty('plan_status')) {
        this.plan_status = initObj.plan_status
      }
      else {
        this.plan_status = false;
      }
      if (initObj.hasOwnProperty('modify_status')) {
        this.modify_status = initObj.modify_status
      }
      else {
        this.modify_status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EgoPlannerResult
    // Serialize message field [planner_goal]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.planner_goal, buffer, bufferOffset);
    // Serialize message field [plan_times]
    bufferOffset = _serializer.int16(obj.plan_times, buffer, bufferOffset);
    // Serialize message field [plan_status]
    bufferOffset = _serializer.bool(obj.plan_status, buffer, bufferOffset);
    // Serialize message field [modify_status]
    bufferOffset = _serializer.bool(obj.modify_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EgoPlannerResult
    let len;
    let data = new EgoPlannerResult(null);
    // Deserialize message field [planner_goal]
    data.planner_goal = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [plan_times]
    data.plan_times = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [plan_status]
    data.plan_status = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [modify_status]
    data.modify_status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'quadrotor_msgs/EgoPlannerResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e6cf40d72e8bfa9282deb6b127bfe25d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #data structure
    geometry_msgs/Vector3 planner_goal
    int16 plan_times
    bool plan_status
    bool modify_status
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new EgoPlannerResult(null);
    if (msg.planner_goal !== undefined) {
      resolved.planner_goal = geometry_msgs.msg.Vector3.Resolve(msg.planner_goal)
    }
    else {
      resolved.planner_goal = new geometry_msgs.msg.Vector3()
    }

    if (msg.plan_times !== undefined) {
      resolved.plan_times = msg.plan_times;
    }
    else {
      resolved.plan_times = 0
    }

    if (msg.plan_status !== undefined) {
      resolved.plan_status = msg.plan_status;
    }
    else {
      resolved.plan_status = false
    }

    if (msg.modify_status !== undefined) {
      resolved.modify_status = msg.modify_status;
    }
    else {
      resolved.modify_status = false
    }

    return resolved;
    }
};

module.exports = EgoPlannerResult;
