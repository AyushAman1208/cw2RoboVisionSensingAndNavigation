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

class Task2ServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ref_object_points = null;
      this.mystery_object_point = null;
    }
    else {
      if (initObj.hasOwnProperty('ref_object_points')) {
        this.ref_object_points = initObj.ref_object_points
      }
      else {
        this.ref_object_points = [];
      }
      if (initObj.hasOwnProperty('mystery_object_point')) {
        this.mystery_object_point = initObj.mystery_object_point
      }
      else {
        this.mystery_object_point = new geometry_msgs.msg.PointStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Task2ServiceRequest
    // Serialize message field [ref_object_points]
    // Serialize the length for message field [ref_object_points]
    bufferOffset = _serializer.uint32(obj.ref_object_points.length, buffer, bufferOffset);
    obj.ref_object_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.PointStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [mystery_object_point]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.mystery_object_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Task2ServiceRequest
    let len;
    let data = new Task2ServiceRequest(null);
    // Deserialize message field [ref_object_points]
    // Deserialize array length for message field [ref_object_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ref_object_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ref_object_points[i] = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [mystery_object_point]
    data.mystery_object_point = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.ref_object_points.forEach((val) => {
      length += geometry_msgs.msg.PointStamped.getMessageSize(val);
    });
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.mystery_object_point);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/Task2ServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b1ec94690a4a67e2e9cfe653f9e4890';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PointStamped[] ref_object_points
    geometry_msgs/PointStamped mystery_object_point
    
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
    const resolved = new Task2ServiceRequest(null);
    if (msg.ref_object_points !== undefined) {
      resolved.ref_object_points = new Array(msg.ref_object_points.length);
      for (let i = 0; i < resolved.ref_object_points.length; ++i) {
        resolved.ref_object_points[i] = geometry_msgs.msg.PointStamped.Resolve(msg.ref_object_points[i]);
      }
    }
    else {
      resolved.ref_object_points = []
    }

    if (msg.mystery_object_point !== undefined) {
      resolved.mystery_object_point = geometry_msgs.msg.PointStamped.Resolve(msg.mystery_object_point)
    }
    else {
      resolved.mystery_object_point = new geometry_msgs.msg.PointStamped()
    }

    return resolved;
    }
};

class Task2ServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mystery_object_num = null;
    }
    else {
      if (initObj.hasOwnProperty('mystery_object_num')) {
        this.mystery_object_num = initObj.mystery_object_num
      }
      else {
        this.mystery_object_num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Task2ServiceResponse
    // Serialize message field [mystery_object_num]
    bufferOffset = _serializer.int64(obj.mystery_object_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Task2ServiceResponse
    let len;
    let data = new Task2ServiceResponse(null);
    // Deserialize message field [mystery_object_num]
    data.mystery_object_num = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/Task2ServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b1e7915ca008eed2a8644edc0612ba6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 mystery_object_num
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Task2ServiceResponse(null);
    if (msg.mystery_object_num !== undefined) {
      resolved.mystery_object_num = msg.mystery_object_num;
    }
    else {
      resolved.mystery_object_num = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: Task2ServiceRequest,
  Response: Task2ServiceResponse,
  md5sum() { return '48fa6a1a8f22948fb54540fdb831dc23'; },
  datatype() { return 'cw2_world_spawner/Task2Service'; }
};
