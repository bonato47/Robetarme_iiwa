// Auto-generated. Do not edit!

// (in-package my_robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MoveUntilGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Min_error = null;
    }
    else {
      if (initObj.hasOwnProperty('Min_error')) {
        this.Min_error = initObj.Min_error
      }
      else {
        this.Min_error = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveUntilGoal
    // Serialize message field [Min_error]
    bufferOffset = _serializer.float64(obj.Min_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveUntilGoal
    let len;
    let data = new MoveUntilGoal(null);
    // Deserialize message field [Min_error]
    data.Min_error = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_robot_msgs/MoveUntilGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '329a6d4b69a6e44522f1d579d9ca32e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #goal 
    float64 Min_error
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveUntilGoal(null);
    if (msg.Min_error !== undefined) {
      resolved.Min_error = msg.Min_error;
    }
    else {
      resolved.Min_error = 0.0
    }

    return resolved;
    }
};

module.exports = MoveUntilGoal;
