// Auto-generated. Do not edit!

// (in-package cuadc_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BucketAimInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.count = null;
      this.valid = null;
      this.bucket_center_x = null;
      this.bucket_center_y = null;
      this.a_aim_x = null;
      this.a_aim_y = null;
      this.b_aim_x = null;
      this.b_aim_y = null;
      this.a_delta_x_px = null;
      this.a_delta_y_px = null;
      this.b_delta_x_px = null;
      this.b_delta_y_px = null;
      this.a_delta_x_m = null;
      this.a_delta_y_m = null;
      this.b_delta_x_m = null;
      this.b_delta_y_m = null;
      this.a_ned_valid = null;
      this.a_ned_n = null;
      this.a_ned_e = null;
      this.a_ned_d = null;
      this.b_ned_valid = null;
      this.b_ned_n = null;
      this.b_ned_e = null;
      this.b_ned_d = null;
      this.confidence = null;
      this.source_class = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0;
      }
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
      if (initObj.hasOwnProperty('bucket_center_x')) {
        this.bucket_center_x = initObj.bucket_center_x
      }
      else {
        this.bucket_center_x = 0;
      }
      if (initObj.hasOwnProperty('bucket_center_y')) {
        this.bucket_center_y = initObj.bucket_center_y
      }
      else {
        this.bucket_center_y = 0;
      }
      if (initObj.hasOwnProperty('a_aim_x')) {
        this.a_aim_x = initObj.a_aim_x
      }
      else {
        this.a_aim_x = 0;
      }
      if (initObj.hasOwnProperty('a_aim_y')) {
        this.a_aim_y = initObj.a_aim_y
      }
      else {
        this.a_aim_y = 0;
      }
      if (initObj.hasOwnProperty('b_aim_x')) {
        this.b_aim_x = initObj.b_aim_x
      }
      else {
        this.b_aim_x = 0;
      }
      if (initObj.hasOwnProperty('b_aim_y')) {
        this.b_aim_y = initObj.b_aim_y
      }
      else {
        this.b_aim_y = 0;
      }
      if (initObj.hasOwnProperty('a_delta_x_px')) {
        this.a_delta_x_px = initObj.a_delta_x_px
      }
      else {
        this.a_delta_x_px = 0.0;
      }
      if (initObj.hasOwnProperty('a_delta_y_px')) {
        this.a_delta_y_px = initObj.a_delta_y_px
      }
      else {
        this.a_delta_y_px = 0.0;
      }
      if (initObj.hasOwnProperty('b_delta_x_px')) {
        this.b_delta_x_px = initObj.b_delta_x_px
      }
      else {
        this.b_delta_x_px = 0.0;
      }
      if (initObj.hasOwnProperty('b_delta_y_px')) {
        this.b_delta_y_px = initObj.b_delta_y_px
      }
      else {
        this.b_delta_y_px = 0.0;
      }
      if (initObj.hasOwnProperty('a_delta_x_m')) {
        this.a_delta_x_m = initObj.a_delta_x_m
      }
      else {
        this.a_delta_x_m = 0.0;
      }
      if (initObj.hasOwnProperty('a_delta_y_m')) {
        this.a_delta_y_m = initObj.a_delta_y_m
      }
      else {
        this.a_delta_y_m = 0.0;
      }
      if (initObj.hasOwnProperty('b_delta_x_m')) {
        this.b_delta_x_m = initObj.b_delta_x_m
      }
      else {
        this.b_delta_x_m = 0.0;
      }
      if (initObj.hasOwnProperty('b_delta_y_m')) {
        this.b_delta_y_m = initObj.b_delta_y_m
      }
      else {
        this.b_delta_y_m = 0.0;
      }
      if (initObj.hasOwnProperty('a_ned_valid')) {
        this.a_ned_valid = initObj.a_ned_valid
      }
      else {
        this.a_ned_valid = false;
      }
      if (initObj.hasOwnProperty('a_ned_n')) {
        this.a_ned_n = initObj.a_ned_n
      }
      else {
        this.a_ned_n = 0.0;
      }
      if (initObj.hasOwnProperty('a_ned_e')) {
        this.a_ned_e = initObj.a_ned_e
      }
      else {
        this.a_ned_e = 0.0;
      }
      if (initObj.hasOwnProperty('a_ned_d')) {
        this.a_ned_d = initObj.a_ned_d
      }
      else {
        this.a_ned_d = 0.0;
      }
      if (initObj.hasOwnProperty('b_ned_valid')) {
        this.b_ned_valid = initObj.b_ned_valid
      }
      else {
        this.b_ned_valid = false;
      }
      if (initObj.hasOwnProperty('b_ned_n')) {
        this.b_ned_n = initObj.b_ned_n
      }
      else {
        this.b_ned_n = 0.0;
      }
      if (initObj.hasOwnProperty('b_ned_e')) {
        this.b_ned_e = initObj.b_ned_e
      }
      else {
        this.b_ned_e = 0.0;
      }
      if (initObj.hasOwnProperty('b_ned_d')) {
        this.b_ned_d = initObj.b_ned_d
      }
      else {
        this.b_ned_d = 0.0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('source_class')) {
        this.source_class = initObj.source_class
      }
      else {
        this.source_class = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BucketAimInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [count]
    bufferOffset = _serializer.int32(obj.count, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [bucket_center_x]
    bufferOffset = _serializer.int32(obj.bucket_center_x, buffer, bufferOffset);
    // Serialize message field [bucket_center_y]
    bufferOffset = _serializer.int32(obj.bucket_center_y, buffer, bufferOffset);
    // Serialize message field [a_aim_x]
    bufferOffset = _serializer.int32(obj.a_aim_x, buffer, bufferOffset);
    // Serialize message field [a_aim_y]
    bufferOffset = _serializer.int32(obj.a_aim_y, buffer, bufferOffset);
    // Serialize message field [b_aim_x]
    bufferOffset = _serializer.int32(obj.b_aim_x, buffer, bufferOffset);
    // Serialize message field [b_aim_y]
    bufferOffset = _serializer.int32(obj.b_aim_y, buffer, bufferOffset);
    // Serialize message field [a_delta_x_px]
    bufferOffset = _serializer.float32(obj.a_delta_x_px, buffer, bufferOffset);
    // Serialize message field [a_delta_y_px]
    bufferOffset = _serializer.float32(obj.a_delta_y_px, buffer, bufferOffset);
    // Serialize message field [b_delta_x_px]
    bufferOffset = _serializer.float32(obj.b_delta_x_px, buffer, bufferOffset);
    // Serialize message field [b_delta_y_px]
    bufferOffset = _serializer.float32(obj.b_delta_y_px, buffer, bufferOffset);
    // Serialize message field [a_delta_x_m]
    bufferOffset = _serializer.float32(obj.a_delta_x_m, buffer, bufferOffset);
    // Serialize message field [a_delta_y_m]
    bufferOffset = _serializer.float32(obj.a_delta_y_m, buffer, bufferOffset);
    // Serialize message field [b_delta_x_m]
    bufferOffset = _serializer.float32(obj.b_delta_x_m, buffer, bufferOffset);
    // Serialize message field [b_delta_y_m]
    bufferOffset = _serializer.float32(obj.b_delta_y_m, buffer, bufferOffset);
    // Serialize message field [a_ned_valid]
    bufferOffset = _serializer.bool(obj.a_ned_valid, buffer, bufferOffset);
    // Serialize message field [a_ned_n]
    bufferOffset = _serializer.float32(obj.a_ned_n, buffer, bufferOffset);
    // Serialize message field [a_ned_e]
    bufferOffset = _serializer.float32(obj.a_ned_e, buffer, bufferOffset);
    // Serialize message field [a_ned_d]
    bufferOffset = _serializer.float32(obj.a_ned_d, buffer, bufferOffset);
    // Serialize message field [b_ned_valid]
    bufferOffset = _serializer.bool(obj.b_ned_valid, buffer, bufferOffset);
    // Serialize message field [b_ned_n]
    bufferOffset = _serializer.float32(obj.b_ned_n, buffer, bufferOffset);
    // Serialize message field [b_ned_e]
    bufferOffset = _serializer.float32(obj.b_ned_e, buffer, bufferOffset);
    // Serialize message field [b_ned_d]
    bufferOffset = _serializer.float32(obj.b_ned_d, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [source_class]
    bufferOffset = _serializer.string(obj.source_class, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BucketAimInfo
    let len;
    let data = new BucketAimInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [count]
    data.count = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [bucket_center_x]
    data.bucket_center_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bucket_center_y]
    data.bucket_center_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [a_aim_x]
    data.a_aim_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [a_aim_y]
    data.a_aim_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [b_aim_x]
    data.b_aim_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [b_aim_y]
    data.b_aim_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [a_delta_x_px]
    data.a_delta_x_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a_delta_y_px]
    data.a_delta_y_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_delta_x_px]
    data.b_delta_x_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_delta_y_px]
    data.b_delta_y_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a_delta_x_m]
    data.a_delta_x_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a_delta_y_m]
    data.a_delta_y_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_delta_x_m]
    data.b_delta_x_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_delta_y_m]
    data.b_delta_y_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a_ned_valid]
    data.a_ned_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [a_ned_n]
    data.a_ned_n = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a_ned_e]
    data.a_ned_e = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a_ned_d]
    data.a_ned_d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_ned_valid]
    data.b_ned_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b_ned_n]
    data.b_ned_n = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_ned_e]
    data.b_ned_e = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_ned_d]
    data.b_ned_d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [source_class]
    data.source_class = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.source_class);
    return length + 95;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cuadc_vision/BucketAimInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '781270f5d3447ff4d4ce3194c04670ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Stable bucket center and physical A/B dropper aim points.
    Header header
    int32 count
    
    # True when detection, depth, intrinsics, pixel aim points and metric deltas are valid.
    # NED validity is reported independently because local_pose may be unavailable.
    bool valid
    
    int32 bucket_center_x
    int32 bucket_center_y
    
    int32 a_aim_x
    int32 a_aim_y
    int32 b_aim_x
    int32 b_aim_y
    
    # Delta convention: delta = aim_point - image_center.
    # Pixel deltas use image axes (x right, y down).
    float32 a_delta_x_px
    float32 a_delta_y_px
    float32 b_delta_x_px
    float32 b_delta_y_px
    
    # Metric deltas are estimated from the current target depth and camera intrinsics.
    float32 a_delta_x_m
    float32 a_delta_y_m
    float32 b_delta_x_m
    float32 b_delta_y_m
    
    # Absolute target positions in the MAVROS local NED frame.
    bool a_ned_valid
    float32 a_ned_n
    float32 a_ned_e
    float32 a_ned_d
    
    bool b_ned_valid
    float32 b_ned_n
    float32 b_ned_e
    float32 b_ned_d
    
    float32 confidence
    string source_class
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BucketAimInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0
    }

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    if (msg.bucket_center_x !== undefined) {
      resolved.bucket_center_x = msg.bucket_center_x;
    }
    else {
      resolved.bucket_center_x = 0
    }

    if (msg.bucket_center_y !== undefined) {
      resolved.bucket_center_y = msg.bucket_center_y;
    }
    else {
      resolved.bucket_center_y = 0
    }

    if (msg.a_aim_x !== undefined) {
      resolved.a_aim_x = msg.a_aim_x;
    }
    else {
      resolved.a_aim_x = 0
    }

    if (msg.a_aim_y !== undefined) {
      resolved.a_aim_y = msg.a_aim_y;
    }
    else {
      resolved.a_aim_y = 0
    }

    if (msg.b_aim_x !== undefined) {
      resolved.b_aim_x = msg.b_aim_x;
    }
    else {
      resolved.b_aim_x = 0
    }

    if (msg.b_aim_y !== undefined) {
      resolved.b_aim_y = msg.b_aim_y;
    }
    else {
      resolved.b_aim_y = 0
    }

    if (msg.a_delta_x_px !== undefined) {
      resolved.a_delta_x_px = msg.a_delta_x_px;
    }
    else {
      resolved.a_delta_x_px = 0.0
    }

    if (msg.a_delta_y_px !== undefined) {
      resolved.a_delta_y_px = msg.a_delta_y_px;
    }
    else {
      resolved.a_delta_y_px = 0.0
    }

    if (msg.b_delta_x_px !== undefined) {
      resolved.b_delta_x_px = msg.b_delta_x_px;
    }
    else {
      resolved.b_delta_x_px = 0.0
    }

    if (msg.b_delta_y_px !== undefined) {
      resolved.b_delta_y_px = msg.b_delta_y_px;
    }
    else {
      resolved.b_delta_y_px = 0.0
    }

    if (msg.a_delta_x_m !== undefined) {
      resolved.a_delta_x_m = msg.a_delta_x_m;
    }
    else {
      resolved.a_delta_x_m = 0.0
    }

    if (msg.a_delta_y_m !== undefined) {
      resolved.a_delta_y_m = msg.a_delta_y_m;
    }
    else {
      resolved.a_delta_y_m = 0.0
    }

    if (msg.b_delta_x_m !== undefined) {
      resolved.b_delta_x_m = msg.b_delta_x_m;
    }
    else {
      resolved.b_delta_x_m = 0.0
    }

    if (msg.b_delta_y_m !== undefined) {
      resolved.b_delta_y_m = msg.b_delta_y_m;
    }
    else {
      resolved.b_delta_y_m = 0.0
    }

    if (msg.a_ned_valid !== undefined) {
      resolved.a_ned_valid = msg.a_ned_valid;
    }
    else {
      resolved.a_ned_valid = false
    }

    if (msg.a_ned_n !== undefined) {
      resolved.a_ned_n = msg.a_ned_n;
    }
    else {
      resolved.a_ned_n = 0.0
    }

    if (msg.a_ned_e !== undefined) {
      resolved.a_ned_e = msg.a_ned_e;
    }
    else {
      resolved.a_ned_e = 0.0
    }

    if (msg.a_ned_d !== undefined) {
      resolved.a_ned_d = msg.a_ned_d;
    }
    else {
      resolved.a_ned_d = 0.0
    }

    if (msg.b_ned_valid !== undefined) {
      resolved.b_ned_valid = msg.b_ned_valid;
    }
    else {
      resolved.b_ned_valid = false
    }

    if (msg.b_ned_n !== undefined) {
      resolved.b_ned_n = msg.b_ned_n;
    }
    else {
      resolved.b_ned_n = 0.0
    }

    if (msg.b_ned_e !== undefined) {
      resolved.b_ned_e = msg.b_ned_e;
    }
    else {
      resolved.b_ned_e = 0.0
    }

    if (msg.b_ned_d !== undefined) {
      resolved.b_ned_d = msg.b_ned_d;
    }
    else {
      resolved.b_ned_d = 0.0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.source_class !== undefined) {
      resolved.source_class = msg.source_class;
    }
    else {
      resolved.source_class = ''
    }

    return resolved;
    }
};

module.exports = BucketAimInfo;
