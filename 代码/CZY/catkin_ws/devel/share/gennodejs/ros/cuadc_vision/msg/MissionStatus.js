// Auto-generated. Do not edit!

// (in-package cuadc_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MissionStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ammo_a = null;
      this.ammo_b = null;
      this.aiming = null;
      this.last_drop = null;
    }
    else {
      if (initObj.hasOwnProperty('ammo_a')) {
        this.ammo_a = initObj.ammo_a
      }
      else {
        this.ammo_a = 0;
      }
      if (initObj.hasOwnProperty('ammo_b')) {
        this.ammo_b = initObj.ammo_b
      }
      else {
        this.ammo_b = 0;
      }
      if (initObj.hasOwnProperty('aiming')) {
        this.aiming = initObj.aiming
      }
      else {
        this.aiming = false;
      }
      if (initObj.hasOwnProperty('last_drop')) {
        this.last_drop = initObj.last_drop
      }
      else {
        this.last_drop = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MissionStatus
    // Serialize message field [ammo_a]
    bufferOffset = _serializer.uint8(obj.ammo_a, buffer, bufferOffset);
    // Serialize message field [ammo_b]
    bufferOffset = _serializer.uint8(obj.ammo_b, buffer, bufferOffset);
    // Serialize message field [aiming]
    bufferOffset = _serializer.bool(obj.aiming, buffer, bufferOffset);
    // Serialize message field [last_drop]
    bufferOffset = _serializer.string(obj.last_drop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MissionStatus
    let len;
    let data = new MissionStatus(null);
    // Deserialize message field [ammo_a]
    data.ammo_a = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ammo_b]
    data.ammo_b = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [aiming]
    data.aiming = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [last_drop]
    data.last_drop = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.last_drop);
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cuadc_vision/MissionStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42a0745a697dd6adddaac7f2c09f5f59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MissionStatus.msg — main.py → detector_node 任务状态同步
    #
    # 用于 detector_node 在画面上显示弹药、瞄准、抛投等任务状态信息。
    
    uint8 ammo_a                # 前抛投器 (A) 剩余弹药数，0 表示无/未挂载
    uint8 ammo_b                # 后抛投器 (B) 剩余弹药数，0 表示无/未挂载
    bool aiming                 # 飞控处于 GUIDED 模式且正在执行对准任务
    string last_drop            # 最近一次抛投的抛投器编号 ("A" / "B" / "")
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MissionStatus(null);
    if (msg.ammo_a !== undefined) {
      resolved.ammo_a = msg.ammo_a;
    }
    else {
      resolved.ammo_a = 0
    }

    if (msg.ammo_b !== undefined) {
      resolved.ammo_b = msg.ammo_b;
    }
    else {
      resolved.ammo_b = 0
    }

    if (msg.aiming !== undefined) {
      resolved.aiming = msg.aiming;
    }
    else {
      resolved.aiming = false
    }

    if (msg.last_drop !== undefined) {
      resolved.last_drop = msg.last_drop;
    }
    else {
      resolved.last_drop = ''
    }

    return resolved;
    }
};

module.exports = MissionStatus;
