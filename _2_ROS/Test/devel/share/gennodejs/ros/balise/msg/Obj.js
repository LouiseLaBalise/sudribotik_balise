// Auto-generated. Do not edit!

// (in-package balise.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Obj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dscript = null;
      this.ident = null;
      this.position = null;
      this.theta = null;
    }
    else {
      if (initObj.hasOwnProperty('dscript')) {
        this.dscript = initObj.dscript
      }
      else {
        this.dscript = '';
      }
      if (initObj.hasOwnProperty('ident')) {
        this.ident = initObj.ident
      }
      else {
        this.ident = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obj
    // Serialize message field [dscript]
    bufferOffset = _serializer.string(obj.dscript, buffer, bufferOffset);
    // Serialize message field [ident]
    bufferOffset = _serializer.uint32(obj.ident, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [theta]
    bufferOffset = _serializer.float64(obj.theta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obj
    let len;
    let data = new Obj(null);
    // Deserialize message field [dscript]
    data.dscript = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ident]
    data.ident = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [theta]
    data.theta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.dscript);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'balise/Obj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f95b3051f8929661f923cb09696172fb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string dscript 
    uint32 ident
    geometry_msgs/Vector3 position
    float64 theta
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
    const resolved = new Obj(null);
    if (msg.dscript !== undefined) {
      resolved.dscript = msg.dscript;
    }
    else {
      resolved.dscript = ''
    }

    if (msg.ident !== undefined) {
      resolved.ident = msg.ident;
    }
    else {
      resolved.ident = 0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Vector3.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Vector3()
    }

    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = 0.0
    }

    return resolved;
    }
};

module.exports = Obj;
