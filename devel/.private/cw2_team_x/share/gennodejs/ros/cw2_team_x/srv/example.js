// Auto-generated. Do not edit!

// (in-package cw2_team_x.srv)


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

class exampleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.example_float = null;
      this.example_int = null;
      this.example_string = null;
      this.example_pose = null;
      this.example_std_vector_of_poses = null;
    }
    else {
      if (initObj.hasOwnProperty('example_float')) {
        this.example_float = initObj.example_float
      }
      else {
        this.example_float = 0.0;
      }
      if (initObj.hasOwnProperty('example_int')) {
        this.example_int = initObj.example_int
      }
      else {
        this.example_int = 0;
      }
      if (initObj.hasOwnProperty('example_string')) {
        this.example_string = initObj.example_string
      }
      else {
        this.example_string = '';
      }
      if (initObj.hasOwnProperty('example_pose')) {
        this.example_pose = initObj.example_pose
      }
      else {
        this.example_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('example_std_vector_of_poses')) {
        this.example_std_vector_of_poses = initObj.example_std_vector_of_poses
      }
      else {
        this.example_std_vector_of_poses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type exampleRequest
    // Serialize message field [example_float]
    bufferOffset = _serializer.float64(obj.example_float, buffer, bufferOffset);
    // Serialize message field [example_int]
    bufferOffset = _serializer.int32(obj.example_int, buffer, bufferOffset);
    // Serialize message field [example_string]
    bufferOffset = _serializer.string(obj.example_string, buffer, bufferOffset);
    // Serialize message field [example_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.example_pose, buffer, bufferOffset);
    // Serialize message field [example_std_vector_of_poses]
    // Serialize the length for message field [example_std_vector_of_poses]
    bufferOffset = _serializer.uint32(obj.example_std_vector_of_poses.length, buffer, bufferOffset);
    obj.example_std_vector_of_poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type exampleRequest
    let len;
    let data = new exampleRequest(null);
    // Deserialize message field [example_float]
    data.example_float = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [example_int]
    data.example_int = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [example_string]
    data.example_string = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [example_pose]
    data.example_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [example_std_vector_of_poses]
    // Deserialize array length for message field [example_std_vector_of_poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.example_std_vector_of_poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.example_std_vector_of_poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.example_string);
    length += 56 * object.example_std_vector_of_poses.length;
    return length + 76;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_team_x/exampleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16f08f6abcc9049e818ebd0c826029fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # this is the request message
    float64 example_float
    int32 example_int
    string example_string
    geometry_msgs/Pose example_pose
    geometry_msgs/Pose[] example_std_vector_of_poses
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new exampleRequest(null);
    if (msg.example_float !== undefined) {
      resolved.example_float = msg.example_float;
    }
    else {
      resolved.example_float = 0.0
    }

    if (msg.example_int !== undefined) {
      resolved.example_int = msg.example_int;
    }
    else {
      resolved.example_int = 0
    }

    if (msg.example_string !== undefined) {
      resolved.example_string = msg.example_string;
    }
    else {
      resolved.example_string = ''
    }

    if (msg.example_pose !== undefined) {
      resolved.example_pose = geometry_msgs.msg.Pose.Resolve(msg.example_pose)
    }
    else {
      resolved.example_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.example_std_vector_of_poses !== undefined) {
      resolved.example_std_vector_of_poses = new Array(msg.example_std_vector_of_poses.length);
      for (let i = 0; i < resolved.example_std_vector_of_poses.length; ++i) {
        resolved.example_std_vector_of_poses[i] = geometry_msgs.msg.Pose.Resolve(msg.example_std_vector_of_poses[i]);
      }
    }
    else {
      resolved.example_std_vector_of_poses = []
    }

    return resolved;
    }
};

class exampleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.example_std_vector_of_points = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('example_std_vector_of_points')) {
        this.example_std_vector_of_points = initObj.example_std_vector_of_points
      }
      else {
        this.example_std_vector_of_points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type exampleResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [example_std_vector_of_points]
    // Serialize the length for message field [example_std_vector_of_points]
    bufferOffset = _serializer.uint32(obj.example_std_vector_of_points.length, buffer, bufferOffset);
    obj.example_std_vector_of_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type exampleResponse
    let len;
    let data = new exampleResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [example_std_vector_of_points]
    // Deserialize array length for message field [example_std_vector_of_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.example_std_vector_of_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.example_std_vector_of_points[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.example_std_vector_of_points.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_team_x/exampleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '21f33296fbcf2cd9ca27d7591c7a3051';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # this is the return message
    bool success
    geometry_msgs/Point[] example_std_vector_of_points
    
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
    const resolved = new exampleResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.example_std_vector_of_points !== undefined) {
      resolved.example_std_vector_of_points = new Array(msg.example_std_vector_of_points.length);
      for (let i = 0; i < resolved.example_std_vector_of_points.length; ++i) {
        resolved.example_std_vector_of_points[i] = geometry_msgs.msg.Point.Resolve(msg.example_std_vector_of_points[i]);
      }
    }
    else {
      resolved.example_std_vector_of_points = []
    }

    return resolved;
    }
};

module.exports = {
  Request: exampleRequest,
  Response: exampleResponse,
  md5sum() { return '978efc3b4d217b6a9fca06d03983e5b2'; },
  datatype() { return 'cw2_team_x/example'; }
};
