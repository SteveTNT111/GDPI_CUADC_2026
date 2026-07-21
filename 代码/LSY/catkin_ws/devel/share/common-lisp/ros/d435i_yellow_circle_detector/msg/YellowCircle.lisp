; Auto-generated. Do not edit!


(cl:in-package d435i_yellow_circle_detector-msg)


;//! \htmlinclude YellowCircle.msg.html

(cl:defclass <YellowCircle> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detected
    :reader detected
    :initarg :detected
    :type cl:boolean
    :initform cl:nil)
   (x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0)
   (depth_m
    :reader depth_m
    :initarg :depth_m
    :type cl:float
    :initform 0.0)
   (raw_depth_m
    :reader raw_depth_m
    :initarg :raw_depth_m
    :type cl:float
    :initform 0.0)
   (diameter_depth_m
    :reader diameter_depth_m
    :initarg :diameter_depth_m
    :type cl:float
    :initform 0.0)
   (calibrated_depth_m
    :reader calibrated_depth_m
    :initarg :calibrated_depth_m
    :type cl:float
    :initform 0.0)
   (depth_source
    :reader depth_source
    :initarg :depth_source
    :type cl:string
    :initform "")
   (center_offset_x
    :reader center_offset_x
    :initarg :center_offset_x
    :type cl:float
    :initform 0.0)
   (center_offset_y
    :reader center_offset_y
    :initarg :center_offset_y
    :type cl:float
    :initform 0.0)
   (position_valid
    :reader position_valid
    :initarg :position_valid
    :type cl:boolean
    :initform cl:nil)
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
   (body_x_m
    :reader body_x_m
    :initarg :body_x_m
    :type cl:float
    :initform 0.0)
   (body_y_m
    :reader body_y_m
    :initarg :body_y_m
    :type cl:float
    :initform 0.0)
   (body_z_m
    :reader body_z_m
    :initarg :body_z_m
    :type cl:float
    :initform 0.0)
   (body_distance_m
    :reader body_distance_m
    :initarg :body_distance_m
    :type cl:float
    :initform 0.0))
)

(cl:defclass YellowCircle (<YellowCircle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YellowCircle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YellowCircle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name d435i_yellow_circle_detector-msg:<YellowCircle> is deprecated: use d435i_yellow_circle_detector-msg:YellowCircle instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:header-val is deprecated.  Use d435i_yellow_circle_detector-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detected-val :lambda-list '(m))
(cl:defmethod detected-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:detected-val is deprecated.  Use d435i_yellow_circle_detector-msg:detected instead.")
  (detected m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:x-val is deprecated.  Use d435i_yellow_circle_detector-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:y-val is deprecated.  Use d435i_yellow_circle_detector-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:radius-val is deprecated.  Use d435i_yellow_circle_detector-msg:radius instead.")
  (radius m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:area-val is deprecated.  Use d435i_yellow_circle_detector-msg:area instead.")
  (area m))

(cl:ensure-generic-function 'depth_m-val :lambda-list '(m))
(cl:defmethod depth_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:depth_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:depth_m instead.")
  (depth_m m))

(cl:ensure-generic-function 'raw_depth_m-val :lambda-list '(m))
(cl:defmethod raw_depth_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:raw_depth_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:raw_depth_m instead.")
  (raw_depth_m m))

(cl:ensure-generic-function 'diameter_depth_m-val :lambda-list '(m))
(cl:defmethod diameter_depth_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:diameter_depth_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:diameter_depth_m instead.")
  (diameter_depth_m m))

(cl:ensure-generic-function 'calibrated_depth_m-val :lambda-list '(m))
(cl:defmethod calibrated_depth_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:calibrated_depth_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:calibrated_depth_m instead.")
  (calibrated_depth_m m))

(cl:ensure-generic-function 'depth_source-val :lambda-list '(m))
(cl:defmethod depth_source-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:depth_source-val is deprecated.  Use d435i_yellow_circle_detector-msg:depth_source instead.")
  (depth_source m))

(cl:ensure-generic-function 'center_offset_x-val :lambda-list '(m))
(cl:defmethod center_offset_x-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:center_offset_x-val is deprecated.  Use d435i_yellow_circle_detector-msg:center_offset_x instead.")
  (center_offset_x m))

(cl:ensure-generic-function 'center_offset_y-val :lambda-list '(m))
(cl:defmethod center_offset_y-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:center_offset_y-val is deprecated.  Use d435i_yellow_circle_detector-msg:center_offset_y instead.")
  (center_offset_y m))

(cl:ensure-generic-function 'position_valid-val :lambda-list '(m))
(cl:defmethod position_valid-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:position_valid-val is deprecated.  Use d435i_yellow_circle_detector-msg:position_valid instead.")
  (position_valid m))

(cl:ensure-generic-function 'camera_x_m-val :lambda-list '(m))
(cl:defmethod camera_x_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:camera_x_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:camera_x_m instead.")
  (camera_x_m m))

(cl:ensure-generic-function 'camera_y_m-val :lambda-list '(m))
(cl:defmethod camera_y_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:camera_y_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:camera_y_m instead.")
  (camera_y_m m))

(cl:ensure-generic-function 'camera_z_m-val :lambda-list '(m))
(cl:defmethod camera_z_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:camera_z_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:camera_z_m instead.")
  (camera_z_m m))

(cl:ensure-generic-function 'distance_m-val :lambda-list '(m))
(cl:defmethod distance_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:distance_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:distance_m instead.")
  (distance_m m))

(cl:ensure-generic-function 'body_x_m-val :lambda-list '(m))
(cl:defmethod body_x_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:body_x_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:body_x_m instead.")
  (body_x_m m))

(cl:ensure-generic-function 'body_y_m-val :lambda-list '(m))
(cl:defmethod body_y_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:body_y_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:body_y_m instead.")
  (body_y_m m))

(cl:ensure-generic-function 'body_z_m-val :lambda-list '(m))
(cl:defmethod body_z_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:body_z_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:body_z_m instead.")
  (body_z_m m))

(cl:ensure-generic-function 'body_distance_m-val :lambda-list '(m))
(cl:defmethod body_distance_m-val ((m <YellowCircle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader d435i_yellow_circle_detector-msg:body_distance_m-val is deprecated.  Use d435i_yellow_circle_detector-msg:body_distance_m instead.")
  (body_distance_m m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YellowCircle>) ostream)
  "Serializes a message object of type '<YellowCircle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'detected) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'raw_depth_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'diameter_depth_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'calibrated_depth_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'depth_source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'depth_source))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'center_offset_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'center_offset_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'position_valid) 1 0)) ostream)
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'body_x_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'body_y_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'body_z_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'body_distance_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YellowCircle>) istream)
  "Deserializes a message object of type '<YellowCircle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'raw_depth_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'diameter_depth_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'calibrated_depth_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth_source) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'depth_source) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_offset_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'center_offset_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'position_valid) (cl:not (cl:zerop (cl:read-byte istream))))
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
    (cl:setf (cl:slot-value msg 'body_x_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'body_y_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'body_z_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'body_distance_m) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YellowCircle>)))
  "Returns string type for a message object of type '<YellowCircle>"
  "d435i_yellow_circle_detector/YellowCircle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YellowCircle)))
  "Returns string type for a message object of type 'YellowCircle"
  "d435i_yellow_circle_detector/YellowCircle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YellowCircle>)))
  "Returns md5sum for a message object of type '<YellowCircle>"
  "546a21369cee59d7afb5df9576c1da21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YellowCircle)))
  "Returns md5sum for a message object of type 'YellowCircle"
  "546a21369cee59d7afb5df9576c1da21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YellowCircle>)))
  "Returns full string definition for message of type '<YellowCircle>"
  (cl:format cl:nil "Header header~%bool detected~%int32 x~%int32 y~%float32 radius~%float32 area~%float32 depth_m~%float32 raw_depth_m~%float32 diameter_depth_m~%float32 calibrated_depth_m~%string depth_source~%float32 center_offset_x~%float32 center_offset_y~%bool position_valid~%float32 camera_x_m~%float32 camera_y_m~%float32 camera_z_m~%float32 distance_m~%float32 body_x_m~%float32 body_y_m~%float32 body_z_m~%float32 body_distance_m~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YellowCircle)))
  "Returns full string definition for message of type 'YellowCircle"
  (cl:format cl:nil "Header header~%bool detected~%int32 x~%int32 y~%float32 radius~%float32 area~%float32 depth_m~%float32 raw_depth_m~%float32 diameter_depth_m~%float32 calibrated_depth_m~%string depth_source~%float32 center_offset_x~%float32 center_offset_y~%bool position_valid~%float32 camera_x_m~%float32 camera_y_m~%float32 camera_z_m~%float32 distance_m~%float32 body_x_m~%float32 body_y_m~%float32 body_z_m~%float32 body_distance_m~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YellowCircle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'depth_source))
     4
     4
     1
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YellowCircle>))
  "Converts a ROS message object to a list"
  (cl:list 'YellowCircle
    (cl:cons ':header (header msg))
    (cl:cons ':detected (detected msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':area (area msg))
    (cl:cons ':depth_m (depth_m msg))
    (cl:cons ':raw_depth_m (raw_depth_m msg))
    (cl:cons ':diameter_depth_m (diameter_depth_m msg))
    (cl:cons ':calibrated_depth_m (calibrated_depth_m msg))
    (cl:cons ':depth_source (depth_source msg))
    (cl:cons ':center_offset_x (center_offset_x msg))
    (cl:cons ':center_offset_y (center_offset_y msg))
    (cl:cons ':position_valid (position_valid msg))
    (cl:cons ':camera_x_m (camera_x_m msg))
    (cl:cons ':camera_y_m (camera_y_m msg))
    (cl:cons ':camera_z_m (camera_z_m msg))
    (cl:cons ':distance_m (distance_m msg))
    (cl:cons ':body_x_m (body_x_m msg))
    (cl:cons ':body_y_m (body_y_m msg))
    (cl:cons ':body_z_m (body_z_m msg))
    (cl:cons ':body_distance_m (body_distance_m msg))
))
