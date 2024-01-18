// Auto-generated. Do not edit!

// (in-package balise_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PositionPx = require('./PositionPx.js');

//-----------------------------------------------------------

class ArrayPositionPx {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.array_of_positionspx = null;
    }
    else {
      if (initObj.hasOwnProperty('array_of_positionspx')) {
        this.array_of_positionspx = initObj.array_of_positionspx
      }
      else {
        this.array_of_positionspx = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArrayPositionPx
    // Serialize message field [array_of_positionspx]
    // Serialize the length for message field [array_of_positionspx]
    bufferOffset = _serializer.uint32(obj.array_of_positionspx.length, buffer, bufferOffset);
    obj.array_of_positionspx.forEach((val) => {
      bufferOffset = PositionPx.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArrayPositionPx
    let len;
    let data = new ArrayPositionPx(null);
    // Deserialize message field [array_of_positionspx]
    // Deserialize array length for message field [array_of_positionspx]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.array_of_positionspx = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.array_of_positionspx[i] = PositionPx.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.array_of_positionspx.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'balise_msgs/ArrayPositionPx';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d42662441de878808f5bfe3b2bdeae4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PositionPx[] array_of_positionspx
    ================================================================================
    MSG: balise_msgs/PositionPx
    int32 x
    int32 y 
    int32 theta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArrayPositionPx(null);
    if (msg.array_of_positionspx !== undefined) {
      resolved.array_of_positionspx = new Array(msg.array_of_positionspx.length);
      for (let i = 0; i < resolved.array_of_positionspx.length; ++i) {
        resolved.array_of_positionspx[i] = PositionPx.Resolve(msg.array_of_positionspx[i]);
      }
    }
    else {
      resolved.array_of_positionspx = []
    }

    return resolved;
    }
};

module.exports = ArrayPositionPx;
