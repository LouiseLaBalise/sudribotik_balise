// Auto-generated. Do not edit!

// (in-package balise_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Score {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.score = null;
    }
    else {
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Score
    // Serialize message field [score]
    bufferOffset = _serializer.int32(obj.score, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Score
    let len;
    let data = new Score(null);
    // Deserialize message field [score]
    data.score = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'balise_msgs/Score';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ed963831938da1fc8151b77dec7741f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 score
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Score(null);
    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0
    }

    return resolved;
    }
};

module.exports = Score;
