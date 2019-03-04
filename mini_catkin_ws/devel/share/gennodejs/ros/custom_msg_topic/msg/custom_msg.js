// Auto-generated. Do not edit!

// (in-package custom_msg_topic.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class custom_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.levels_of_anger = null;
      this.min_lidar_data = null;
      this.Avoidance_Classification_group = null;
      this.Speed_change_of_obstacle = null;
    }
    else {
      if (initObj.hasOwnProperty('levels_of_anger')) {
        this.levels_of_anger = initObj.levels_of_anger
      }
      else {
        this.levels_of_anger = 0.0;
      }
      if (initObj.hasOwnProperty('min_lidar_data')) {
        this.min_lidar_data = initObj.min_lidar_data
      }
      else {
        this.min_lidar_data = 0.0;
      }
      if (initObj.hasOwnProperty('Avoidance_Classification_group')) {
        this.Avoidance_Classification_group = initObj.Avoidance_Classification_group
      }
      else {
        this.Avoidance_Classification_group = [];
      }
      if (initObj.hasOwnProperty('Speed_change_of_obstacle')) {
        this.Speed_change_of_obstacle = initObj.Speed_change_of_obstacle
      }
      else {
        this.Speed_change_of_obstacle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type custom_msg
    // Serialize message field [levels_of_anger]
    bufferOffset = _serializer.float32(obj.levels_of_anger, buffer, bufferOffset);
    // Serialize message field [min_lidar_data]
    bufferOffset = _serializer.float32(obj.min_lidar_data, buffer, bufferOffset);
    // Serialize message field [Avoidance_Classification_group]
    bufferOffset = _arraySerializer.float32(obj.Avoidance_Classification_group, buffer, bufferOffset, null);
    // Serialize message field [Speed_change_of_obstacle]
    bufferOffset = _serializer.float32(obj.Speed_change_of_obstacle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type custom_msg
    let len;
    let data = new custom_msg(null);
    // Deserialize message field [levels_of_anger]
    data.levels_of_anger = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_lidar_data]
    data.min_lidar_data = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Avoidance_Classification_group]
    data.Avoidance_Classification_group = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [Speed_change_of_obstacle]
    data.Speed_change_of_obstacle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.Avoidance_Classification_group.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg_topic/custom_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '56c5740684da4cec8bf62501dd7b9504';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 levels_of_anger
    float32 min_lidar_data
    float32[] Avoidance_Classification_group
    float32 Speed_change_of_obstacle 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new custom_msg(null);
    if (msg.levels_of_anger !== undefined) {
      resolved.levels_of_anger = msg.levels_of_anger;
    }
    else {
      resolved.levels_of_anger = 0.0
    }

    if (msg.min_lidar_data !== undefined) {
      resolved.min_lidar_data = msg.min_lidar_data;
    }
    else {
      resolved.min_lidar_data = 0.0
    }

    if (msg.Avoidance_Classification_group !== undefined) {
      resolved.Avoidance_Classification_group = msg.Avoidance_Classification_group;
    }
    else {
      resolved.Avoidance_Classification_group = []
    }

    if (msg.Speed_change_of_obstacle !== undefined) {
      resolved.Speed_change_of_obstacle = msg.Speed_change_of_obstacle;
    }
    else {
      resolved.Speed_change_of_obstacle = 0.0
    }

    return resolved;
    }
};

module.exports = custom_msg;
