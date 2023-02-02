// Auto-generated. Do not edit!

// (in-package iiwa_tools.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GetJacobiansRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_angles = null;
      this.joint_velocities = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_angles')) {
        this.joint_angles = initObj.joint_angles
      }
      else {
        this.joint_angles = [];
      }
      if (initObj.hasOwnProperty('joint_velocities')) {
        this.joint_velocities = initObj.joint_velocities
      }
      else {
        this.joint_velocities = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetJacobiansRequest
    // Serialize message field [joint_angles]
    bufferOffset = _arraySerializer.float64(obj.joint_angles, buffer, bufferOffset, null);
    // Serialize message field [joint_velocities]
    bufferOffset = _arraySerializer.float64(obj.joint_velocities, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJacobiansRequest
    let len;
    let data = new GetJacobiansRequest(null);
    // Deserialize message field [joint_angles]
    data.joint_angles = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocities]
    data.joint_velocities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_angles.length;
    length += 8 * object.joint_velocities.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'iiwa_tools/GetJacobiansRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac8a55797891c85d442a15b1722805ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] joint_angles
    float64[] joint_velocities
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJacobiansRequest(null);
    if (msg.joint_angles !== undefined) {
      resolved.joint_angles = msg.joint_angles;
    }
    else {
      resolved.joint_angles = []
    }

    if (msg.joint_velocities !== undefined) {
      resolved.joint_velocities = msg.joint_velocities;
    }
    else {
      resolved.joint_velocities = []
    }

    return resolved;
    }
};

class GetJacobiansResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jacobian = null;
      this.jacobian_deriv = null;
    }
    else {
      if (initObj.hasOwnProperty('jacobian')) {
        this.jacobian = initObj.jacobian
      }
      else {
        this.jacobian = new std_msgs.msg.Float64MultiArray();
      }
      if (initObj.hasOwnProperty('jacobian_deriv')) {
        this.jacobian_deriv = initObj.jacobian_deriv
      }
      else {
        this.jacobian_deriv = new std_msgs.msg.Float64MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetJacobiansResponse
    // Serialize message field [jacobian]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.jacobian, buffer, bufferOffset);
    // Serialize message field [jacobian_deriv]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.jacobian_deriv, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetJacobiansResponse
    let len;
    let data = new GetJacobiansResponse(null);
    // Deserialize message field [jacobian]
    data.jacobian = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [jacobian_deriv]
    data.jacobian_deriv = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.jacobian);
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.jacobian_deriv);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'iiwa_tools/GetJacobiansResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '683c65a58cbeeab9573076ade456ee9f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray jacobian
    std_msgs/Float64MultiArray jacobian_deriv
    
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetJacobiansResponse(null);
    if (msg.jacobian !== undefined) {
      resolved.jacobian = std_msgs.msg.Float64MultiArray.Resolve(msg.jacobian)
    }
    else {
      resolved.jacobian = new std_msgs.msg.Float64MultiArray()
    }

    if (msg.jacobian_deriv !== undefined) {
      resolved.jacobian_deriv = std_msgs.msg.Float64MultiArray.Resolve(msg.jacobian_deriv)
    }
    else {
      resolved.jacobian_deriv = new std_msgs.msg.Float64MultiArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetJacobiansRequest,
  Response: GetJacobiansResponse,
  md5sum() { return '7cd7b210d0e70924367a510d05e781d4'; },
  datatype() { return 'iiwa_tools/GetJacobians'; }
};
