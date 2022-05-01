// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class los {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robotName = null;
    }
    else {
      if (initObj.hasOwnProperty('robotName')) {
        this.robotName = initObj.robotName
      }
      else {
        this.robotName = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type los
    // Serialize message field [robotName]
    bufferOffset = _serializer.string(obj.robotName, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type los
    let len;
    let data = new los(null);
    // Deserialize message field [robotName]
    data.robotName = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.robotName);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/los';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e4ce7af4736710e228ed1cbe6f009e7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string robotName
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new los(null);
    if (msg.robotName !== undefined) {
      resolved.robotName = msg.robotName;
    }
    else {
      resolved.robotName = ''
    }

    return resolved;
    }
};

module.exports = los;
