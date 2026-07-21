// Auto-generated. Do not edit!

// (in-package d435i_yellow_circle_detector.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class YellowCircle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detected = null;
      this.x = null;
      this.y = null;
      this.radius = null;
      this.area = null;
      this.depth_m = null;
      this.raw_depth_m = null;
      this.diameter_depth_m = null;
      this.calibrated_depth_m = null;
      this.depth_source = null;
      this.center_offset_x = null;
      this.center_offset_y = null;
      this.position_valid = null;
      this.camera_x_m = null;
      this.camera_y_m = null;
      this.camera_z_m = null;
      this.distance_m = null;
      this.body_x_m = null;
      this.body_y_m = null;
      this.body_z_m = null;
      this.body_distance_m = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detected')) {
        this.detected = initObj.detected
      }
      else {
        this.detected = false;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('area')) {
        this.area = initObj.area
      }
      else {
        this.area = 0.0;
      }
      if (initObj.hasOwnProperty('depth_m')) {
        this.depth_m = initObj.depth_m
      }
      else {
        this.depth_m = 0.0;
      }
      if (initObj.hasOwnProperty('raw_depth_m')) {
        this.raw_depth_m = initObj.raw_depth_m
      }
      else {
        this.raw_depth_m = 0.0;
      }
      if (initObj.hasOwnProperty('diameter_depth_m')) {
        this.diameter_depth_m = initObj.diameter_depth_m
      }
      else {
        this.diameter_depth_m = 0.0;
      }
      if (initObj.hasOwnProperty('calibrated_depth_m')) {
        this.calibrated_depth_m = initObj.calibrated_depth_m
      }
      else {
        this.calibrated_depth_m = 0.0;
      }
      if (initObj.hasOwnProperty('depth_source')) {
        this.depth_source = initObj.depth_source
      }
      else {
        this.depth_source = '';
      }
      if (initObj.hasOwnProperty('center_offset_x')) {
        this.center_offset_x = initObj.center_offset_x
      }
      else {
        this.center_offset_x = 0.0;
      }
      if (initObj.hasOwnProperty('center_offset_y')) {
        this.center_offset_y = initObj.center_offset_y
      }
      else {
        this.center_offset_y = 0.0;
      }
      if (initObj.hasOwnProperty('position_valid')) {
        this.position_valid = initObj.position_valid
      }
      else {
        this.position_valid = false;
      }
      if (initObj.hasOwnProperty('camera_x_m')) {
        this.camera_x_m = initObj.camera_x_m
      }
      else {
        this.camera_x_m = 0.0;
      }
      if (initObj.hasOwnProperty('camera_y_m')) {
        this.camera_y_m = initObj.camera_y_m
      }
      else {
        this.camera_y_m = 0.0;
      }
      if (initObj.hasOwnProperty('camera_z_m')) {
        this.camera_z_m = initObj.camera_z_m
      }
      else {
        this.camera_z_m = 0.0;
      }
      if (initObj.hasOwnProperty('distance_m')) {
        this.distance_m = initObj.distance_m
      }
      else {
        this.distance_m = 0.0;
      }
      if (initObj.hasOwnProperty('body_x_m')) {
        this.body_x_m = initObj.body_x_m
      }
      else {
        this.body_x_m = 0.0;
      }
      if (initObj.hasOwnProperty('body_y_m')) {
        this.body_y_m = initObj.body_y_m
      }
      else {
        this.body_y_m = 0.0;
      }
      if (initObj.hasOwnProperty('body_z_m')) {
        this.body_z_m = initObj.body_z_m
      }
      else {
        this.body_z_m = 0.0;
      }
      if (initObj.hasOwnProperty('body_distance_m')) {
        this.body_distance_m = initObj.body_distance_m
      }
      else {
        this.body_distance_m = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YellowCircle
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detected]
    bufferOffset = _serializer.bool(obj.detected, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.int32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.int32(obj.y, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float32(obj.radius, buffer, bufferOffset);
    // Serialize message field [area]
    bufferOffset = _serializer.float32(obj.area, buffer, bufferOffset);
    // Serialize message field [depth_m]
    bufferOffset = _serializer.float32(obj.depth_m, buffer, bufferOffset);
    // Serialize message field [raw_depth_m]
    bufferOffset = _serializer.float32(obj.raw_depth_m, buffer, bufferOffset);
    // Serialize message field [diameter_depth_m]
    bufferOffset = _serializer.float32(obj.diameter_depth_m, buffer, bufferOffset);
    // Serialize message field [calibrated_depth_m]
    bufferOffset = _serializer.float32(obj.calibrated_depth_m, buffer, bufferOffset);
    // Serialize message field [depth_source]
    bufferOffset = _serializer.string(obj.depth_source, buffer, bufferOffset);
    // Serialize message field [center_offset_x]
    bufferOffset = _serializer.float32(obj.center_offset_x, buffer, bufferOffset);
    // Serialize message field [center_offset_y]
    bufferOffset = _serializer.float32(obj.center_offset_y, buffer, bufferOffset);
    // Serialize message field [position_valid]
    bufferOffset = _serializer.bool(obj.position_valid, buffer, bufferOffset);
    // Serialize message field [camera_x_m]
    bufferOffset = _serializer.float32(obj.camera_x_m, buffer, bufferOffset);
    // Serialize message field [camera_y_m]
    bufferOffset = _serializer.float32(obj.camera_y_m, buffer, bufferOffset);
    // Serialize message field [camera_z_m]
    bufferOffset = _serializer.float32(obj.camera_z_m, buffer, bufferOffset);
    // Serialize message field [distance_m]
    bufferOffset = _serializer.float32(obj.distance_m, buffer, bufferOffset);
    // Serialize message field [body_x_m]
    bufferOffset = _serializer.float32(obj.body_x_m, buffer, bufferOffset);
    // Serialize message field [body_y_m]
    bufferOffset = _serializer.float32(obj.body_y_m, buffer, bufferOffset);
    // Serialize message field [body_z_m]
    bufferOffset = _serializer.float32(obj.body_z_m, buffer, bufferOffset);
    // Serialize message field [body_distance_m]
    bufferOffset = _serializer.float32(obj.body_distance_m, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YellowCircle
    let len;
    let data = new YellowCircle(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detected]
    data.detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [area]
    data.area = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [depth_m]
    data.depth_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [raw_depth_m]
    data.raw_depth_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [diameter_depth_m]
    data.diameter_depth_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [calibrated_depth_m]
    data.calibrated_depth_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [depth_source]
    data.depth_source = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [center_offset_x]
    data.center_offset_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [center_offset_y]
    data.center_offset_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [position_valid]
    data.position_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [camera_x_m]
    data.camera_x_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [camera_y_m]
    data.camera_y_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [camera_z_m]
    data.camera_z_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance_m]
    data.distance_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [body_x_m]
    data.body_x_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [body_y_m]
    data.body_y_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [body_z_m]
    data.body_z_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [body_distance_m]
    data.body_distance_m = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.depth_source);
    return length + 78;
  }

  static datatype() {
    // Returns string type for a message object
    return 'd435i_yellow_circle_detector/YellowCircle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '546a21369cee59d7afb5df9576c1da21';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool detected
    int32 x
    int32 y
    float32 radius
    float32 area
    float32 depth_m
    float32 raw_depth_m
    float32 diameter_depth_m
    float32 calibrated_depth_m
    string depth_source
    float32 center_offset_x
    float32 center_offset_y
    bool position_valid
    float32 camera_x_m
    float32 camera_y_m
    float32 camera_z_m
    float32 distance_m
    float32 body_x_m
    float32 body_y_m
    float32 body_z_m
    float32 body_distance_m
    
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
    const resolved = new YellowCircle(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detected !== undefined) {
      resolved.detected = msg.detected;
    }
    else {
      resolved.detected = false
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.area !== undefined) {
      resolved.area = msg.area;
    }
    else {
      resolved.area = 0.0
    }

    if (msg.depth_m !== undefined) {
      resolved.depth_m = msg.depth_m;
    }
    else {
      resolved.depth_m = 0.0
    }

    if (msg.raw_depth_m !== undefined) {
      resolved.raw_depth_m = msg.raw_depth_m;
    }
    else {
      resolved.raw_depth_m = 0.0
    }

    if (msg.diameter_depth_m !== undefined) {
      resolved.diameter_depth_m = msg.diameter_depth_m;
    }
    else {
      resolved.diameter_depth_m = 0.0
    }

    if (msg.calibrated_depth_m !== undefined) {
      resolved.calibrated_depth_m = msg.calibrated_depth_m;
    }
    else {
      resolved.calibrated_depth_m = 0.0
    }

    if (msg.depth_source !== undefined) {
      resolved.depth_source = msg.depth_source;
    }
    else {
      resolved.depth_source = ''
    }

    if (msg.center_offset_x !== undefined) {
      resolved.center_offset_x = msg.center_offset_x;
    }
    else {
      resolved.center_offset_x = 0.0
    }

    if (msg.center_offset_y !== undefined) {
      resolved.center_offset_y = msg.center_offset_y;
    }
    else {
      resolved.center_offset_y = 0.0
    }

    if (msg.position_valid !== undefined) {
      resolved.position_valid = msg.position_valid;
    }
    else {
      resolved.position_valid = false
    }

    if (msg.camera_x_m !== undefined) {
      resolved.camera_x_m = msg.camera_x_m;
    }
    else {
      resolved.camera_x_m = 0.0
    }

    if (msg.camera_y_m !== undefined) {
      resolved.camera_y_m = msg.camera_y_m;
    }
    else {
      resolved.camera_y_m = 0.0
    }

    if (msg.camera_z_m !== undefined) {
      resolved.camera_z_m = msg.camera_z_m;
    }
    else {
      resolved.camera_z_m = 0.0
    }

    if (msg.distance_m !== undefined) {
      resolved.distance_m = msg.distance_m;
    }
    else {
      resolved.distance_m = 0.0
    }

    if (msg.body_x_m !== undefined) {
      resolved.body_x_m = msg.body_x_m;
    }
    else {
      resolved.body_x_m = 0.0
    }

    if (msg.body_y_m !== undefined) {
      resolved.body_y_m = msg.body_y_m;
    }
    else {
      resolved.body_y_m = 0.0
    }

    if (msg.body_z_m !== undefined) {
      resolved.body_z_m = msg.body_z_m;
    }
    else {
      resolved.body_z_m = 0.0
    }

    if (msg.body_distance_m !== undefined) {
      resolved.body_distance_m = msg.body_distance_m;
    }
    else {
      resolved.body_distance_m = 0.0
    }

    return resolved;
    }
};

module.exports = YellowCircle;
