// Auto-generated. Do not edit!

// (in-package chatroom.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ChatMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sender = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('sender')) {
        this.sender = initObj.sender
      }
      else {
        this.sender = '';
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChatMessage
    // Serialize message field [sender]
    bufferOffset = _serializer.string(obj.sender, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChatMessage
    let len;
    let data = new ChatMessage(null);
    // Deserialize message field [sender]
    data.sender = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.sender);
    length += _getByteLength(object.message);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chatroom/ChatMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '985ef42f02a992c58d5de728edeb121e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string sender
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChatMessage(null);
    if (msg.sender !== undefined) {
      resolved.sender = msg.sender;
    }
    else {
      resolved.sender = ''
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = ChatMessage;
