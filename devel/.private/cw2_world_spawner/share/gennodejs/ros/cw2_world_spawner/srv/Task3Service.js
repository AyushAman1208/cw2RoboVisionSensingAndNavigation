// Auto-generated. Do not edit!

// (in-package cw2_world_spawner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Task3ServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Task3ServiceRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Task3ServiceRequest
    let len;
    let data = new Task3ServiceRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/Task3ServiceRequest';
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
    const resolved = new Task3ServiceRequest(null);
    return resolved;
    }
};

class Task3ServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.total_num_shapes = null;
      this.num_most_common_shape = null;
      this.most_common_shape_vector = null;
    }
    else {
      if (initObj.hasOwnProperty('total_num_shapes')) {
        this.total_num_shapes = initObj.total_num_shapes
      }
      else {
        this.total_num_shapes = 0;
      }
      if (initObj.hasOwnProperty('num_most_common_shape')) {
        this.num_most_common_shape = initObj.num_most_common_shape
      }
      else {
        this.num_most_common_shape = 0;
      }
      if (initObj.hasOwnProperty('most_common_shape_vector')) {
        this.most_common_shape_vector = initObj.most_common_shape_vector
      }
      else {
        this.most_common_shape_vector = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Task3ServiceResponse
    // Serialize message field [total_num_shapes]
    bufferOffset = _serializer.int64(obj.total_num_shapes, buffer, bufferOffset);
    // Serialize message field [num_most_common_shape]
    bufferOffset = _serializer.int64(obj.num_most_common_shape, buffer, bufferOffset);
    // Serialize message field [most_common_shape_vector]
    bufferOffset = _arraySerializer.int64(obj.most_common_shape_vector, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Task3ServiceResponse
    let len;
    let data = new Task3ServiceResponse(null);
    // Deserialize message field [total_num_shapes]
    data.total_num_shapes = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [num_most_common_shape]
    data.num_most_common_shape = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [most_common_shape_vector]
    data.most_common_shape_vector = _arrayDeserializer.int64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.most_common_shape_vector.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/Task3ServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4cdcbf342ebe5d2a91b198b8ea6a615';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 total_num_shapes
    int64 num_most_common_shape
    int64[] most_common_shape_vector
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Task3ServiceResponse(null);
    if (msg.total_num_shapes !== undefined) {
      resolved.total_num_shapes = msg.total_num_shapes;
    }
    else {
      resolved.total_num_shapes = 0
    }

    if (msg.num_most_common_shape !== undefined) {
      resolved.num_most_common_shape = msg.num_most_common_shape;
    }
    else {
      resolved.num_most_common_shape = 0
    }

    if (msg.most_common_shape_vector !== undefined) {
      resolved.most_common_shape_vector = msg.most_common_shape_vector;
    }
    else {
      resolved.most_common_shape_vector = []
    }

    return resolved;
    }
};

module.exports = {
  Request: Task3ServiceRequest,
  Response: Task3ServiceResponse,
  md5sum() { return 'a4cdcbf342ebe5d2a91b198b8ea6a615'; },
  datatype() { return 'cw2_world_spawner/Task3Service'; }
};
