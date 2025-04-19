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

class Obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.s_start = null;
      this.s_end = null;
      this.d_right = null;
      this.d_left = null;
      this.is_actually_a_gap = null;
      this.s_center = null;
      this.d_center = null;
      this.size = null;
      this.vs = null;
      this.vd = null;
      this.s_var = null;
      this.d_var = null;
      this.vs_var = null;
      this.vd_var = null;
      this.is_static = null;
      this.is_visible = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('s_start')) {
        this.s_start = initObj.s_start
      }
      else {
        this.s_start = 0.0;
      }
      if (initObj.hasOwnProperty('s_end')) {
        this.s_end = initObj.s_end
      }
      else {
        this.s_end = 0.0;
      }
      if (initObj.hasOwnProperty('d_right')) {
        this.d_right = initObj.d_right
      }
      else {
        this.d_right = 0.0;
      }
      if (initObj.hasOwnProperty('d_left')) {
        this.d_left = initObj.d_left
      }
      else {
        this.d_left = 0.0;
      }
      if (initObj.hasOwnProperty('is_actually_a_gap')) {
        this.is_actually_a_gap = initObj.is_actually_a_gap
      }
      else {
        this.is_actually_a_gap = false;
      }
      if (initObj.hasOwnProperty('s_center')) {
        this.s_center = initObj.s_center
      }
      else {
        this.s_center = 0.0;
      }
      if (initObj.hasOwnProperty('d_center')) {
        this.d_center = initObj.d_center
      }
      else {
        this.d_center = 0.0;
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0.0;
      }
      if (initObj.hasOwnProperty('vs')) {
        this.vs = initObj.vs
      }
      else {
        this.vs = 0.0;
      }
      if (initObj.hasOwnProperty('vd')) {
        this.vd = initObj.vd
      }
      else {
        this.vd = 0.0;
      }
      if (initObj.hasOwnProperty('s_var')) {
        this.s_var = initObj.s_var
      }
      else {
        this.s_var = 0.0;
      }
      if (initObj.hasOwnProperty('d_var')) {
        this.d_var = initObj.d_var
      }
      else {
        this.d_var = 0.0;
      }
      if (initObj.hasOwnProperty('vs_var')) {
        this.vs_var = initObj.vs_var
      }
      else {
        this.vs_var = 0.0;
      }
      if (initObj.hasOwnProperty('vd_var')) {
        this.vd_var = initObj.vd_var
      }
      else {
        this.vd_var = 0.0;
      }
      if (initObj.hasOwnProperty('is_static')) {
        this.is_static = initObj.is_static
      }
      else {
        this.is_static = false;
      }
      if (initObj.hasOwnProperty('is_visible')) {
        this.is_visible = initObj.is_visible
      }
      else {
        this.is_visible = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obstacle
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [s_start]
    bufferOffset = _serializer.float64(obj.s_start, buffer, bufferOffset);
    // Serialize message field [s_end]
    bufferOffset = _serializer.float64(obj.s_end, buffer, bufferOffset);
    // Serialize message field [d_right]
    bufferOffset = _serializer.float64(obj.d_right, buffer, bufferOffset);
    // Serialize message field [d_left]
    bufferOffset = _serializer.float64(obj.d_left, buffer, bufferOffset);
    // Serialize message field [is_actually_a_gap]
    bufferOffset = _serializer.bool(obj.is_actually_a_gap, buffer, bufferOffset);
    // Serialize message field [s_center]
    bufferOffset = _serializer.float64(obj.s_center, buffer, bufferOffset);
    // Serialize message field [d_center]
    bufferOffset = _serializer.float64(obj.d_center, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = _serializer.float64(obj.size, buffer, bufferOffset);
    // Serialize message field [vs]
    bufferOffset = _serializer.float64(obj.vs, buffer, bufferOffset);
    // Serialize message field [vd]
    bufferOffset = _serializer.float64(obj.vd, buffer, bufferOffset);
    // Serialize message field [s_var]
    bufferOffset = _serializer.float64(obj.s_var, buffer, bufferOffset);
    // Serialize message field [d_var]
    bufferOffset = _serializer.float64(obj.d_var, buffer, bufferOffset);
    // Serialize message field [vs_var]
    bufferOffset = _serializer.float64(obj.vs_var, buffer, bufferOffset);
    // Serialize message field [vd_var]
    bufferOffset = _serializer.float64(obj.vd_var, buffer, bufferOffset);
    // Serialize message field [is_static]
    bufferOffset = _serializer.bool(obj.is_static, buffer, bufferOffset);
    // Serialize message field [is_visible]
    bufferOffset = _serializer.bool(obj.is_visible, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obstacle
    let len;
    let data = new Obstacle(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [s_start]
    data.s_start = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [s_end]
    data.s_end = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_right]
    data.d_right = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_left]
    data.d_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [is_actually_a_gap]
    data.is_actually_a_gap = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [s_center]
    data.s_center = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_center]
    data.d_center = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vs]
    data.vs = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vd]
    data.vd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [s_var]
    data.s_var = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_var]
    data.d_var = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vs_var]
    data.vs_var = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vd_var]
    data.vd_var = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [is_static]
    data.is_static = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_visible]
    data.is_visible = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 111;
  }

  static datatype() {
    // Returns string type for a message object
    return 'f110_msgs/Obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '778e7375210689bb1da0fcff69e149b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 id
    
    float64 s_start
    float64 s_end
    float64 d_right           # defined as right bound of the obstacle
    float64 d_left            # defined as left bound of the obstacle
    bool is_actually_a_gap    # used by the frenet planner to choos points through which we want to go
    float64 s_center
    float64 d_center
    float64 size
    float64 vs
    float64 vd
    float64 s_var
    float64 d_var
    float64 vs_var
    float64 vd_var
    bool is_static
    bool is_visible
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Obstacle(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.s_start !== undefined) {
      resolved.s_start = msg.s_start;
    }
    else {
      resolved.s_start = 0.0
    }

    if (msg.s_end !== undefined) {
      resolved.s_end = msg.s_end;
    }
    else {
      resolved.s_end = 0.0
    }

    if (msg.d_right !== undefined) {
      resolved.d_right = msg.d_right;
    }
    else {
      resolved.d_right = 0.0
    }

    if (msg.d_left !== undefined) {
      resolved.d_left = msg.d_left;
    }
    else {
      resolved.d_left = 0.0
    }

    if (msg.is_actually_a_gap !== undefined) {
      resolved.is_actually_a_gap = msg.is_actually_a_gap;
    }
    else {
      resolved.is_actually_a_gap = false
    }

    if (msg.s_center !== undefined) {
      resolved.s_center = msg.s_center;
    }
    else {
      resolved.s_center = 0.0
    }

    if (msg.d_center !== undefined) {
      resolved.d_center = msg.d_center;
    }
    else {
      resolved.d_center = 0.0
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0.0
    }

    if (msg.vs !== undefined) {
      resolved.vs = msg.vs;
    }
    else {
      resolved.vs = 0.0
    }

    if (msg.vd !== undefined) {
      resolved.vd = msg.vd;
    }
    else {
      resolved.vd = 0.0
    }

    if (msg.s_var !== undefined) {
      resolved.s_var = msg.s_var;
    }
    else {
      resolved.s_var = 0.0
    }

    if (msg.d_var !== undefined) {
      resolved.d_var = msg.d_var;
    }
    else {
      resolved.d_var = 0.0
    }

    if (msg.vs_var !== undefined) {
      resolved.vs_var = msg.vs_var;
    }
    else {
      resolved.vs_var = 0.0
    }

    if (msg.vd_var !== undefined) {
      resolved.vd_var = msg.vd_var;
    }
    else {
      resolved.vd_var = 0.0
    }

    if (msg.is_static !== undefined) {
      resolved.is_static = msg.is_static;
    }
    else {
      resolved.is_static = false
    }

    if (msg.is_visible !== undefined) {
      resolved.is_visible = msg.is_visible;
    }
    else {
      resolved.is_visible = false
    }

    return resolved;
    }
};

module.exports = Obstacle;
