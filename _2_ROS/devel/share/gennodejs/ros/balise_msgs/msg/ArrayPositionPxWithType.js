// Auto-generated. Do not edit!

// (in-package balise_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PositionPxWithType = require('./PositionPxWithType.js');

//-----------------------------------------------------------

class ArrayPositionPxWithType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.array_of_positionspx_with_type = null;
    }
    else {
      if (initObj.hasOwnProperty('array_of_positionspx_with_type')) {
        this.array_of_positionspx_with_type = initObj.array_of_positionspx_with_type
      }
      else {
        this.array_of_positionspx_with_type = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArrayPositionPxWithType
    // Serialize message field [array_of_positionspx_with_type]
    // Serialize the length for message field [array_of_positionspx_with_type]
    bufferOffset = _serializer.uint32(obj.array_of_positionspx_with_type.length, buffer, bufferOffset);
    obj.array_of_positionspx_with_type.forEach((val) => {
      bufferOffset = PositionPxWithType.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArrayPositionPxWithType
    let len;
    let data = new ArrayPositionPxWithType(null);
    // Deserialize message field [array_of_positionspx_with_type]
    // Deserialize array length for message field [array_of_positionspx_with_type]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.array_of_positionspx_with_type = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.array_of_positionspx_with_type[i] = PositionPxWithType.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.array_of_positionspx_with_type.forEach((val) => {
      length += PositionPxWithType.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'balise_msgs/ArrayPositionPxWithType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb0df7150251fed4c0b95568bc63c1f4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PositionPxWithType[] array_of_positionspx_with_type
    ================================================================================
    MSG: balise_msgs/PositionPxWithType
    int32 x
    int32 y 
    int32 theta
    string type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArrayPositionPxWithType(null);
    if (msg.array_of_positionspx_with_type !== undefined) {
      resolved.array_of_positionspx_with_type = new Array(msg.array_of_positionspx_with_type.length);
      for (let i = 0; i < resolved.array_of_positionspx_with_type.length; ++i) {
        resolved.array_of_positionspx_with_type[i] = PositionPxWithType.Resolve(msg.array_of_positionspx_with_type[i]);
      }
    }
    else {
      resolved.array_of_positionspx_with_type = []
    }

    return resolved;
    }
};

module.exports = ArrayPositionPxWithType;
