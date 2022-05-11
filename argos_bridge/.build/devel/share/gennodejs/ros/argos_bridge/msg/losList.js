// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let los = require('./los.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class losList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.n = null;
      this.robots = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('n')) {
        this.n = initObj.n
      }
      else {
        this.n = 0;
      }
      if (initObj.hasOwnProperty('robots')) {
        this.robots = initObj.robots
      }
      else {
        this.robots = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type losList
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [n]
    bufferOffset = _serializer.int32(obj.n, buffer, bufferOffset);
    // Serialize message field [robots]
    // Serialize the length for message field [robots]
    bufferOffset = _serializer.uint32(obj.robots.length, buffer, bufferOffset);
    obj.robots.forEach((val) => {
      bufferOffset = los.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type losList
    let len;
    let data = new losList(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [n]
    data.n = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [robots]
    // Deserialize array length for message field [robots]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.robots = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.robots[i] = los.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.robots.forEach((val) => {
      length += los.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/losList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3c2be5cd3cebc9db2ce990dc41ef741c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 n
    los[] robots
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: argos_bridge/los
    string robotName
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new losList(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.n !== undefined) {
      resolved.n = msg.n;
    }
    else {
      resolved.n = 0
    }

    if (msg.robots !== undefined) {
      resolved.robots = new Array(msg.robots.length);
      for (let i = 0; i < resolved.robots.length; ++i) {
        resolved.robots[i] = los.Resolve(msg.robots[i]);
      }
    }
    else {
      resolved.robots = []
    }

    return resolved;
    }
};

module.exports = losList;
