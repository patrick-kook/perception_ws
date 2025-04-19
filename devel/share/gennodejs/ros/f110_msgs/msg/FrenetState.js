// Auto-generated. Do not edit!

// (in-package f110_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FrenetState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.s = null;
      this.d = null;
      this.v = null;
    }
    else {
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = 0.0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FrenetState
    // Serialize message field [s]
    bufferOffset = _serializer.float64(obj.s, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float64(obj.d, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.float64(obj.v, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrenetState
    let len;
    let data = new FrenetState(null);
    // Deserialize message field [s]
    data.s = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'f110_msgs/FrenetState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f35f65d8c6f765f27afccad48d013c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 s       # The longitudinal position along the Frenet path
    float64 d       # The lateral distance from the Frenet path
    float64 v       # The longitudinal velocity (m/s)
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FrenetState(null);
    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = 0.0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0.0
    }

    return resolved;
    }
};

module.exports = FrenetState;
