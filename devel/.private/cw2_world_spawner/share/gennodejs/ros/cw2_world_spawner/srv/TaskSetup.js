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

class TaskSetupRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_index = null;
    }
    else {
      if (initObj.hasOwnProperty('task_index')) {
        this.task_index = initObj.task_index
      }
      else {
        this.task_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskSetupRequest
    // Serialize message field [task_index]
    bufferOffset = _serializer.int8(obj.task_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskSetupRequest
    let len;
    let data = new TaskSetupRequest(null);
    // Deserialize message field [task_index]
    data.task_index = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/TaskSetupRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '713f50eb6979d6bda1389d9b25c308e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 task_index
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TaskSetupRequest(null);
    if (msg.task_index !== undefined) {
      resolved.task_index = msg.task_index;
    }
    else {
      resolved.task_index = 0
    }

    return resolved;
    }
};

class TaskSetupResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskSetupResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskSetupResponse
    let len;
    let data = new TaskSetupResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cw2_world_spawner/TaskSetupResponse';
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
    const resolved = new TaskSetupResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: TaskSetupRequest,
  Response: TaskSetupResponse,
  md5sum() { return '713f50eb6979d6bda1389d9b25c308e0'; },
  datatype() { return 'cw2_world_spawner/TaskSetup'; }
};
