; Auto-generated. Do not edit!


(cl:in-package d435i_yellow_circle_detector-msg)


;//! \htmlinclude MissionTarget.msg.html

(cl:defclass <MissionTarget> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil)
   (mission_stage
    :reader mission_stage
    :initarg :mission_stage
    :type cl:string
    :initform "")
   (target_type
    :reader target_type
    :initarg :target_type
    :type cl:string
    :initform "")
   (class_name
    :reader class_name
    :initarg :class_name
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (center_x
    :reader center_x
    :initarg :center_x
    :type cl:integer
    :initform 0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:integer
    :initform 0)
   (camera_x_m
    :reader camera_x_m
    :initarg :camera_x_m
    :type cl:float
    :initform 0.0)
   (camera_y_m
    :reader camera_y_m
    :initarg :camera_y_m
    :type cl:float
    :initform 0.0)
   (camera_z_m
    :reader camera_z_m
    :initarg :camera_z_m
    :type cl:float
    :initform 0.0)
   (distance_m
    :reader distance_m
    :initarg :distance_m
    :type cl:float
    :initform 0.0)
   (bbox_width_m
    :reader bbox_width_m
    :initarg :bbox_width_m
    :type cl:float
    :initform 0.0)
   (bbox_height_m
    :reader bbox_height_m
    :initarg :bbox_height_m
    :type cl:float
    :initform 0.0)
   (nominal_diameter_m
    :reader nominal_diameter_m
    :initarg :nominal_diameter_m
    :type cl:float
    :initform 0.0)
   (diameter_class
    :reader diameter_class
    :initarg :diameter_class
    :type cl:string
    :initform "")
   (zone_hint
    :reader zone_hint
    :initarg :zone_hint
    :type cl:string
    :initform "")
   (a_zone_radius_m
    :reader a_zone_radius_m
    :initarg :a_zone_radius_m
    :type cl:float
    :initform 0.0)
   (b_zone_radius_m
    :reader b_zone_radius_m
    :initarg :b_zone_radius_m
    :type cl:float
    :initform 0.0)
   (action_hint
    :reader action_hint
    :initarg :action_hint
    :type cl:string
    :initform ""))
)

(cl:defclass MissionTarget (<MissionTarget>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MissionTarget>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MissionTarget)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name d435i_yellow_circle_detector-msg:<MissionTarget> is deprecated: use d435i_yellow_circle_detector-msg:MissionTarget instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:header-val is deprecated.  Use d435i_yellow_circle_detector-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:valid-val is deprecated.  Use d435i_yellow_circle_detector-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'mission_stage-val :lambda-list '(m))
(cl:defmethod mission_stage-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:mission_stage-val is deprecated.  Use d435i_yellow_circle_detector-msg:mission_stage instead.")
  (mission_stage m))

(cl:ensure-generic-function 'target_type-val :lambda-list '(m))
(cl:defmethod target_type-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:target_type-val is deprecated.  Use d435i_yellow_circle_detector-msg:target_type instead.")
  (target_type m))

(cl:ensure-generic-function 'class_name-val :lambda-list '(m))
(cl:defmethod class_name-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:class_name-val is deprecated.  Use d435i_yellow_circle_detector-msg:class_name instead.")
  (class_name m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:confidence-val is deprecated.  Use d435i_yellow_circle_detector-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:center_x-val is deprecated.  Use d435i_yellow_circle_detector-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:center_y-val is deprecated.  Use d435i_yellow_circle_detector-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'camera_x_m-val :lambda-list '(m))
(cl:defmethod camera_x_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:camera_x_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:camera_x_m instead.")
  (camera_x_m m))

(cl:ensure-generic-function 'camera_y_m-val :lambda-list '(m))
(cl:defmethod camera_y_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:camera_y_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:camera_y_m instead.")
  (camera_y_m m))

(cl:ensure-generic-function 'camera_z_m-val :lambda-list '(m))
(cl:defmethod camera_z_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:camera_z_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:camera_z_m instead.")
  (camera_z_m m))

(cl:ensure-generic-function 'distance_m-val :lambda-list '(m))
(cl:defmethod distance_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:distance_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:distance_m instead.")
  (distance_m m))

(cl:ensure-generic-function 'bbox_width_m-val :lambda-list '(m))
(cl:defmethod bbox_width_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:bbox_width_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:bbox_width_m instead.")
  (bbox_width_m m))

(cl:ensure-generic-function 'bbox_height_m-val :lambda-list '(m))
(cl:defmethod bbox_height_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:bbox_height_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:bbox_height_m instead.")
  (bbox_height_m m))

(cl:ensure-generic-function 'nominal_diameter_m-val :lambda-list '(m))
(cl:defmethod nominal_diameter_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:nominal_diameter_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:nominal_diameter_m instead.")
  (nominal_diameter_m m))

(cl:ensure-generic-function 'diameter_class-val :lambda-list '(m))
(cl:defmethod diameter_class-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:diameter_class-val is deprecated.  Use d435i_yellow_circle_detector-msg:diameter_class instead.")
  (diameter_class m))

(cl:ensure-generic-function 'zone_hint-val :lambda-list '(m))
(cl:defmethod zone_hint-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:zone_hint-val is deprecated.  Use d435i_yellow_circle_detector-msg:zone_hint instead.")
  (zone_hint m))

(cl:ensure-generic-function 'a_zone_radius_m-val :lambda-list '(m))
(cl:defmethod a_zone_radius_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:a_zone_radius_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:a_zone_radius_m instead.")
  (a_zone_radius_m m))

(cl:ensure-generic-function 'b_zone_radius_m-val :lambda-list '(m))
(cl:defmethod b_zone_radius_m-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:b_zone_radius_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:b_zone_radius_m instead.")
  (b_zone_radius_m m))

(cl:ensure-generic-function 'action_hint-val :lambda-list '(m))
(cl:defmethod action_hint-val ((m <MissionTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:action_hint-val is deprecated.  Use d435i_yellow_circle_detector-msg:action_hint instead.")
  (action_hint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MissionTarget>) ostream)
  "Serializes a message object of type '<MissionTarget>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mission_stage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mission_stage))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target_type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'class_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'class_name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'center_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'center_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'camera_x_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'camera_y_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'camera_z_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_width_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbox_height_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'nominal_diameter_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'diameter_class))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'diameter_class))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'zone_hint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'zone_hint))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_zone_radius_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_zone_radius_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action_hint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action_hint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MissionTarget>) istream)
  "Deserializes a message object of type '<MissionTarget>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mission_stage) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mission_stage) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'class_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'class_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'camera_x_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'camera_y_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'camera_z_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_width_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbox_height_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'nominal_diameter_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'diameter_class) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'diameter_class) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'zone_hint) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'zone_hint) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_zone_radius_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_zone_radius_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action_hint) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action_hint) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MissionTarget>)))
  "Returns string type for a message object of type '<MissionTarget>"
  "d435i_yellow_circle_detector/MissionTarget")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MissionTarget)))
  "Returns string type for a message object of type 'MissionTarget"
  "d435i_yellow_circle_detector/MissionTarget")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MissionTarget>)))
  "Returns md5sum for a message object of type '<MissionTarget>"
  "9e275971ddc71593e97b9bdc197f8cad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MissionTarget)))
  "Returns md5sum for a message object of type 'MissionTarget"
  "9e275971ddc71593e97b9bdc197f8cad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MissionTarget>)))
  "Returns full string definition for message of type '<MissionTarget>"
  (cl:format cl:nil "Header header~%bool valid~%string mission_stage~%string target_type~%string class_name~%float32 confidence~%int32 center_x~%int32 center_y~%float32 camera_x_m~%float32 camera_y_m~%float32 camera_z_m~%float32 distance_m~%float32 bbox_width_m~%float32 bbox_height_m~%float32 nominal_diameter_m~%string diameter_class~%string zone_hint~%float32 a_zone_radius_m~%float32 b_zone_radius_m~%string action_hint~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MissionTarget)))
  "Returns full string definition for message of type 'MissionTarget"
  (cl:format cl:nil "Header header~%bool valid~%string mission_stage~%string target_type~%string class_name~%float32 confidence~%int32 center_x~%int32 center_y~%float32 camera_x_m~%float32 camera_y_m~%float32 camera_z_m~%float32 distance_m~%float32 bbox_width_m~%float32 bbox_height_m~%float32 nominal_diameter_m~%string diameter_class~%string zone_hint~%float32 a_zone_radius_m~%float32 b_zone_radius_m~%string action_hint~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MissionTarget>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'mission_stage))
     4 (cl:length (cl:slot-value msg 'target_type))
     4 (cl:length (cl:slot-value msg 'class_name))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'diameter_class))
     4 (cl:length (cl:slot-value msg 'zone_hint))
     4
     4
     4 (cl:length (cl:slot-value msg 'action_hint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MissionTarget>))
  "Converts a ROS message object to a list"
  (cl:list 'MissionTarget
    (cl:cons ':header (header msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':mission_stage (mission_stage msg))
    (cl:cons ':target_type (target_type msg))
    (cl:cons ':class_name (class_name msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':camera_x_m (camera_x_m msg))
    (cl:cons ':camera_y_m (camera_y_m msg))
    (cl:cons ':camera_z_m (camera_z_m msg))
    (cl:cons ':distance_m (distance_m msg))
    (cl:cons ':bbox_width_m (bbox_width_m msg))
    (cl:cons ':bbox_height_m (bbox_height_m msg))
    (cl:cons ':nominal_diameter_m (nominal_diameter_m msg))
    (cl:cons ':diameter_class (diameter_class msg))
    (cl:cons ':zone_hint (zone_hint msg))
    (cl:cons ':a_zone_radius_m (a_zone_radius_m msg))
    (cl:cons ':b_zone_radius_m (b_zone_radius_m msg))
    (cl:cons ':action_hint (action_hint msg))
))
