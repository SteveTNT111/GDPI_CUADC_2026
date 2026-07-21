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

class GeoTarget {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.valid = null;
      this.status = null;
      this.source_topic = null;
      this.class_name = null;
      this.confidence = null;
      this.center_x = null;
      this.center_y = null;
      this.camera_x_m = null;
      this.camera_y_m = null;
      this.camera_z_m = null;
      this.body_x_m = null;
      this.body_y_m = null;
      this.body_z_m = null;
      this.enu_east_m = null;
      this.enu_north_m = null;
      this.enu_up_m = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
      if (initObj.hasOwnProperty('source_topic')) {
        this.source_topic = initObj.source_topic
      }
      else {
        this.source_topic = '';
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
      if (initObj.hasOwnProperty('enu_east_m')) {
        this.enu_east_m = initObj.enu_east_m
      }
      else {
        this.enu_east_m = 0.0;
      }
      if (initObj.hasOwnProperty('enu_north_m')) {
        this.enu_north_m = initObj.enu_north_m
      }
      else {
        this.enu_north_m = 0.0;
      }
      if (initObj.hasOwnProperty('enu_up_m')) {
        this.enu_up_m = initObj.enu_up_m
      }
      else {
        this.enu_up_m = 0.0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GeoTarget
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    // Serialize message field [source_topic]
    bufferOffset = _serializer.string(obj.source_topic, buffer, bufferOffset);
    // Serialize message field [class_name]
    bufferOffset = _serializer.string(obj.class_name, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [center_x]
    bufferOffset = _serializer.int32(obj.center_x, buffer, bufferOffset);
    // Serialize message field [center_y]
    bufferOffset = _serializer.int32(obj.center_y, buffer, bufferOffset);
    // Serialize message field [camera_x_m]
    bufferOffset = _serializer.float32(obj.camera_x_m, buffer, bufferOffset);
    // Serialize message field [camera_y_m]
    bufferOffset = _serializer.float32(obj.camera_y_m, buffer, bufferOffset);
    // Serialize message field [camera_z_m]
    bufferOffset = _serializer.float32(obj.camera_z_m, buffer, bufferOffset);
    // Serialize message field [body_x_m]
    bufferOffset = _serializer.float32(obj.body_x_m, buffer, bufferOffset);
    // Serialize message field [body_y_m]
    bufferOffset = _serializer.float32(obj.body_y_m, buffer, bufferOffset);
    // Serialize message field [body_z_m]
    bufferOffset = _serializer.float32(obj.body_z_m, buffer, bufferOffset);
    // Serialize message field [enu_east_m]
    bufferOffset = _serializer.float32(obj.enu_east_m, buffer, bufferOffset);
    // Serialize message field [enu_north_m]
    bufferOffset = _serializer.float32(obj.enu_north_m, buffer, bufferOffset);
    // Serialize message field [enu_up_m]
    bufferOffset = _serializer.float32(obj.enu_up_m, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GeoTarget
    let len;
    let data = new GeoTarget(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [source_topic]
    data.source_topic = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [class_name]
    data.class_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [center_x]
    data.center_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [center_y]
    data.center_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [camera_x_m]
    data.camera_x_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [camera_y_m]
    data.camera_y_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [camera_z_m]
    data.camera_z_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [body_x_m]
    data.body_x_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [body_y_m]
    data.body_y_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [body_z_m]
    data.body_z_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [enu_east_m]
    data.enu_east_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [enu_north_m]
    data.enu_north_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [enu_up_m]
    data.enu_up_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.status);
    length += _getByteLength(object.source_topic);
    length += _getByteLength(object.class_name);
    return length + 85;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cuadc_vision/GeoTarget';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '97435bcc3e1e35e6ba97ca5a2f45aeba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool valid
    string status
    string source_topic
    string class_name
    float32 confidence
    int32 center_x
    int32 center_y
    float32 camera_x_m
    float32 camera_y_m
    float32 camera_z_m
    float32 body_x_m
    float32 body_y_m
    float32 body_z_m
    float32 enu_east_m
    float32 enu_north_m
    float32 enu_up_m
    float64 latitude
    float64 longitude
    float64 altitude
    
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
    const resolved = new GeoTarget(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    if (msg.source_topic !== undefined) {
      resolved.source_topic = msg.source_topic;
    }
    else {
      resolved.source_topic = ''
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

    if (msg.enu_east_m !== undefined) {
      resolved.enu_east_m = msg.enu_east_m;
    }
    else {
      resolved.enu_east_m = 0.0
    }

    if (msg.enu_north_m !== undefined) {
      resolved.enu_north_m = msg.enu_north_m;
    }
    else {
      resolved.enu_north_m = 0.0
    }

    if (msg.enu_up_m !== undefined) {
      resolved.enu_up_m = msg.enu_up_m;
    }
    else {
      resolved.enu_up_m = 0.0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    return resolved;
    }
};

module.exports = GeoTarget;
