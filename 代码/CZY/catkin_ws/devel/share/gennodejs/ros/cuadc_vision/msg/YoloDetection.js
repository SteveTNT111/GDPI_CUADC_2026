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

class YoloDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detected = null;
      this.class_id = null;
      this.class_name = null;
      this.confidence = null;
      this.x_min = null;
      this.y_min = null;
      this.x_max = null;
      this.y_max = null;
      this.center_x = null;
      this.center_y = null;
      this.depth_m = null;
      this.position_valid = null;
      this.camera_x_m = null;
      this.camera_y_m = null;
      this.camera_z_m = null;
      this.distance_m = null;
      this.bbox_width_m = null;
      this.bbox_height_m = null;
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
      if (initObj.hasOwnProperty('class_id')) {
        this.class_id = initObj.class_id
      }
      else {
        this.class_id = 0;
      }
      if (initObj.hasOwnProperty('class_name')) {
        this.class_name = initObj.class_name
      }
      else {
        this.class_name = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('x_min')) {
        this.x_min = initObj.x_min
      }
      else {
        this.x_min = 0;
      }
      if (initObj.hasOwnProperty('y_min')) {
        this.y_min = initObj.y_min
      }
      else {
        this.y_min = 0;
      }
      if (initObj.hasOwnProperty('x_max')) {
        this.x_max = initObj.x_max
      }
      else {
        this.x_max = 0;
      }
      if (initObj.hasOwnProperty('y_max')) {
        this.y_max = initObj.y_max
      }
      else {
        this.y_max = 0;
      }
      if (initObj.hasOwnProperty('center_x')) {
        this.center_x = initObj.center_x
      }
      else {
        this.center_x = 0;
      }
      if (initObj.hasOwnProperty('center_y')) {
        this.center_y = initObj.center_y
      }
      else {
        this.center_y = 0;
      }
      if (initObj.hasOwnProperty('depth_m')) {
        this.depth_m = initObj.depth_m
      }
      else {
        this.depth_m = 0.0;
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
      if (initObj.hasOwnProperty('bbox_width_m')) {
        this.bbox_width_m = initObj.bbox_width_m
      }
      else {
        this.bbox_width_m = 0.0;
      }
      if (initObj.hasOwnProperty('bbox_height_m')) {
        this.bbox_height_m = initObj.bbox_height_m
      }
      else {
        this.bbox_height_m = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type YoloDetection
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detected]
    bufferOffset = _serializer.bool(obj.detected, buffer, bufferOffset);
    // Serialize message field [class_id]
    bufferOffset = _serializer.int32(obj.class_id, buffer, bufferOffset);
    // Serialize message field [class_name]
    bufferOffset = _serializer.string(obj.class_name, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [x_min]
    bufferOffset = _serializer.int32(obj.x_min, buffer, bufferOffset);
    // Serialize message field [y_min]
    bufferOffset = _serializer.int32(obj.y_min, buffer, bufferOffset);
    // Serialize message field [x_max]
    bufferOffset = _serializer.int32(obj.x_max, buffer, bufferOffset);
    // Serialize message field [y_max]
    bufferOffset = _serializer.int32(obj.y_max, buffer, bufferOffset);
    // Serialize message field [center_x]
    bufferOffset = _serializer.int32(obj.center_x, buffer, bufferOffset);
    // Serialize message field [center_y]
    bufferOffset = _serializer.int32(obj.center_y, buffer, bufferOffset);
    // Serialize message field [depth_m]
    bufferOffset = _serializer.float32(obj.depth_m, buffer, bufferOffset);
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
    // Serialize message field [bbox_width_m]
    bufferOffset = _serializer.float32(obj.bbox_width_m, buffer, bufferOffset);
    // Serialize message field [bbox_height_m]
    bufferOffset = _serializer.float32(obj.bbox_height_m, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type YoloDetection
    let len;
    let data = new YoloDetection(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detected]
    data.detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [class_id]
    data.class_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [class_name]
    data.class_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_min]
    data.x_min = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_min]
    data.y_min = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x_max]
    data.x_max = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_max]
    data.y_max = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [center_x]
    data.center_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [center_y]
    data.center_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [depth_m]
    data.depth_m = _deserializer.float32(buffer, bufferOffset);
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
    // Deserialize message field [bbox_width_m]
    data.bbox_width_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_height_m]
    data.bbox_height_m = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.class_name);
    return length + 66;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cuadc_vision/YoloDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c97190c54505fa4fe0a0e57bb2112989';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool detected
    int32 class_id
    string class_name
    float32 confidence
    int32 x_min
    int32 y_min
    int32 x_max
    int32 y_max
    int32 center_x
    int32 center_y
    float32 depth_m
    bool position_valid
    float32 camera_x_m
    float32 camera_y_m
    float32 camera_z_m
    float32 distance_m
    float32 bbox_width_m
    float32 bbox_height_m
    
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
    const resolved = new YoloDetection(null);
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

    if (msg.class_id !== undefined) {
      resolved.class_id = msg.class_id;
    }
    else {
      resolved.class_id = 0
    }

    if (msg.class_name !== undefined) {
      resolved.class_name = msg.class_name;
    }
    else {
      resolved.class_name = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.x_min !== undefined) {
      resolved.x_min = msg.x_min;
    }
    else {
      resolved.x_min = 0
    }

    if (msg.y_min !== undefined) {
      resolved.y_min = msg.y_min;
    }
    else {
      resolved.y_min = 0
    }

    if (msg.x_max !== undefined) {
      resolved.x_max = msg.x_max;
    }
    else {
      resolved.x_max = 0
    }

    if (msg.y_max !== undefined) {
      resolved.y_max = msg.y_max;
    }
    else {
      resolved.y_max = 0
    }

    if (msg.center_x !== undefined) {
      resolved.center_x = msg.center_x;
    }
    else {
      resolved.center_x = 0
    }

    if (msg.center_y !== undefined) {
      resolved.center_y = msg.center_y;
    }
    else {
      resolved.center_y = 0
    }

    if (msg.depth_m !== undefined) {
      resolved.depth_m = msg.depth_m;
    }
    else {
      resolved.depth_m = 0.0
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

    if (msg.bbox_width_m !== undefined) {
      resolved.bbox_width_m = msg.bbox_width_m;
    }
    else {
      resolved.bbox_width_m = 0.0
    }

    if (msg.bbox_height_m !== undefined) {
      resolved.bbox_height_m = msg.bbox_height_m;
    }
    else {
      resolved.bbox_height_m = 0.0
    }

    return resolved;
    }
};

module.exports = YoloDetection;
