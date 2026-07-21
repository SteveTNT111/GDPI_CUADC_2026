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

class MissionTarget {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.valid = null;
      this.mission_stage = null;
      this.target_type = null;
      this.class_name = null;
      this.confidence = null;
      this.center_x = null;
      this.center_y = null;
      this.camera_x_m = null;
      this.camera_y_m = null;
      this.camera_z_m = null;
      this.distance_m = null;
      this.bbox_width_m = null;
      this.bbox_height_m = null;
      this.nominal_diameter_m = null;
      this.diameter_class = null;
      this.zone_hint = null;
      this.a_zone_radius_m = null;
      this.b_zone_radius_m = null;
      this.action_hint = null;
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
      if (initObj.hasOwnProperty('mission_stage')) {
        this.mission_stage = initObj.mission_stage
      }
      else {
        this.mission_stage = '';
      }
      if (initObj.hasOwnProperty('target_type')) {
        this.target_type = initObj.target_type
      }
      else {
        this.target_type = '';
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
      if (initObj.hasOwnProperty('nominal_diameter_m')) {
        this.nominal_diameter_m = initObj.nominal_diameter_m
      }
      else {
        this.nominal_diameter_m = 0.0;
      }
      if (initObj.hasOwnProperty('diameter_class')) {
        this.diameter_class = initObj.diameter_class
      }
      else {
        this.diameter_class = '';
      }
      if (initObj.hasOwnProperty('zone_hint')) {
        this.zone_hint = initObj.zone_hint
      }
      else {
        this.zone_hint = '';
      }
      if (initObj.hasOwnProperty('a_zone_radius_m')) {
        this.a_zone_radius_m = initObj.a_zone_radius_m
      }
      else {
        this.a_zone_radius_m = 0.0;
      }
      if (initObj.hasOwnProperty('b_zone_radius_m')) {
        this.b_zone_radius_m = initObj.b_zone_radius_m
      }
      else {
        this.b_zone_radius_m = 0.0;
      }
      if (initObj.hasOwnProperty('action_hint')) {
        this.action_hint = initObj.action_hint
      }
      else {
        this.action_hint = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MissionTarget
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [mission_stage]
    bufferOffset = _serializer.string(obj.mission_stage, buffer, bufferOffset);
    // Serialize message field [target_type]
    bufferOffset = _serializer.string(obj.target_type, buffer, bufferOffset);
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
    // Serialize message field [distance_m]
    bufferOffset = _serializer.float32(obj.distance_m, buffer, bufferOffset);
    // Serialize message field [bbox_width_m]
    bufferOffset = _serializer.float32(obj.bbox_width_m, buffer, bufferOffset);
    // Serialize message field [bbox_height_m]
    bufferOffset = _serializer.float32(obj.bbox_height_m, buffer, bufferOffset);
    // Serialize message field [nominal_diameter_m]
    bufferOffset = _serializer.float32(obj.nominal_diameter_m, buffer, bufferOffset);
    // Serialize message field [diameter_class]
    bufferOffset = _serializer.string(obj.diameter_class, buffer, bufferOffset);
    // Serialize message field [zone_hint]
    bufferOffset = _serializer.string(obj.zone_hint, buffer, bufferOffset);
    // Serialize message field [a_zone_radius_m]
    bufferOffset = _serializer.float32(obj.a_zone_radius_m, buffer, bufferOffset);
    // Serialize message field [b_zone_radius_m]
    bufferOffset = _serializer.float32(obj.b_zone_radius_m, buffer, bufferOffset);
    // Serialize message field [action_hint]
    bufferOffset = _serializer.string(obj.action_hint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MissionTarget
    let len;
    let data = new MissionTarget(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mission_stage]
    data.mission_stage = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_type]
    data.target_type = _deserializer.string(buffer, bufferOffset);
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
    // Deserialize message field [distance_m]
    data.distance_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_width_m]
    data.bbox_width_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbox_height_m]
    data.bbox_height_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [nominal_diameter_m]
    data.nominal_diameter_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [diameter_class]
    data.diameter_class = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [zone_hint]
    data.zone_hint = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [a_zone_radius_m]
    data.a_zone_radius_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [b_zone_radius_m]
    data.b_zone_radius_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [action_hint]
    data.action_hint = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.mission_stage);
    length += _getByteLength(object.target_type);
    length += _getByteLength(object.class_name);
    length += _getByteLength(object.diameter_class);
    length += _getByteLength(object.zone_hint);
    length += _getByteLength(object.action_hint);
    return length + 73;
  }

  static datatype() {
    // Returns string type for a message object
    return 'd435i_yellow_circle_detector/MissionTarget';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e275971ddc71593e97b9bdc197f8cad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool valid
    string mission_stage
    string target_type
    string class_name
    float32 confidence
    int32 center_x
    int32 center_y
    float32 camera_x_m
    float32 camera_y_m
    float32 camera_z_m
    float32 distance_m
    float32 bbox_width_m
    float32 bbox_height_m
    float32 nominal_diameter_m
    string diameter_class
    string zone_hint
    float32 a_zone_radius_m
    float32 b_zone_radius_m
    string action_hint
    
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
    const resolved = new MissionTarget(null);
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

    if (msg.mission_stage !== undefined) {
      resolved.mission_stage = msg.mission_stage;
    }
    else {
      resolved.mission_stage = ''
    }

    if (msg.target_type !== undefined) {
      resolved.target_type = msg.target_type;
    }
    else {
      resolved.target_type = ''
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

    if (msg.nominal_diameter_m !== undefined) {
      resolved.nominal_diameter_m = msg.nominal_diameter_m;
    }
    else {
      resolved.nominal_diameter_m = 0.0
    }

    if (msg.diameter_class !== undefined) {
      resolved.diameter_class = msg.diameter_class;
    }
    else {
      resolved.diameter_class = ''
    }

    if (msg.zone_hint !== undefined) {
      resolved.zone_hint = msg.zone_hint;
    }
    else {
      resolved.zone_hint = ''
    }

    if (msg.a_zone_radius_m !== undefined) {
      resolved.a_zone_radius_m = msg.a_zone_radius_m;
    }
    else {
      resolved.a_zone_radius_m = 0.0
    }

    if (msg.b_zone_radius_m !== undefined) {
      resolved.b_zone_radius_m = msg.b_zone_radius_m;
    }
    else {
      resolved.b_zone_radius_m = 0.0
    }

    if (msg.action_hint !== undefined) {
      resolved.action_hint = msg.action_hint;
    }
    else {
      resolved.action_hint = ''
    }

    return resolved;
    }
};

module.exports = MissionTarget;
