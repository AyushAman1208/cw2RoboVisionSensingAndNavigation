// Auto-generated. Do not edit!

// (in-package cw2_world_spawner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class Task1ServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_point = null;
      this.goal_point = null;
      this.shape_type = null;
    }
    else {
      if (initObj.hasOwnProperty('object_point')) {
        this.object_point = initObj.object_point
      }
      else {
        this.object_point = new geometry_msgs.msg.PointStamped();
      }
      if (initObj.hasOwnProperty('goal_point')) {
        this.goal_point = initObj.goal_point
      }
      else {
        this.goal_point = new geometry_msgs.msg.PointStamped();
      }
      if (initObj.hasOwnProperty('shape_type')) {
        this.shape_type = initObj.shape_type
      }
      else {
        this.shape_type = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Task1ServiceRequest
    // Serialize message field [object_point]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.object_point, buffer, bufferOffset);
    // Serialize message field [goal_point]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.goal_point, buffer, bufferOffset);
    // Serialize message field [shape_type]
    bufferOffset = _serializer.string(obj.shape_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Task1ServiceRequest
    let len;
    let data = new Task1ServiceRequest(null);
    // Deserialize message field [object_point]
    data.object_point = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_point]
    data.goal_point = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [shape_type]
    data.shape_type = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.object_point);
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.goal_point);
    length += _getByteLength(object.shape_type);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/Task1ServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ee66ea1bcc1704a8551b6f7b84e9cf9e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PointStamped object_point
    geometry_msgs/PointStamped goal_point
    string shape_type
    
    ================================================================================
    MSG: geometry_msgs/PointStamped
    # This represents a Point with reference coordinate frame and timestamp
    Header header
    Point point
    
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new Task1ServiceRequest(null);
    if (msg.object_point !== undefined) {
      resolved.object_point = geometry_msgs.msg.PointStamped.Resolve(msg.object_point)
    }
    else {
      resolved.object_point = new geometry_msgs.msg.PointStamped()
    }

    if (msg.goal_point !== undefined) {
      resolved.goal_point = geometry_msgs.msg.PointStamped.Resolve(msg.goal_point)
    }
    else {
      resolved.goal_point = new geometry_msgs.msg.PointStamped()
    }

    if (msg.shape_type !== undefined) {
      resolved.shape_type = msg.shape_type;
    }
    else {
      resolved.shape_type = ''
    }

    return resolved;
    }
};

class Task1ServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Task1ServiceResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Task1ServiceResponse
    let len;
    let data = new Task1ServiceResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/Task1ServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Task1ServiceResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: Task1ServiceRequest,
  Response: Task1ServiceResponse,
  md5sum() { return 'ee66ea1bcc1704a8551b6f7b84e9cf9e'; },
  datatype() { return 'cw2_world_spawner/Task1Service'; }
};
