; Auto-generated. Do not edit!


(cl:in-package cuadc_vision-msg)


;//! \htmlinclude BucketInfo.msg.html

(cl:defclass <BucketInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0)
   (delta_x
    :reader delta_x
    :initarg :delta_x
    :type cl:float
    :initform 0.0)
   (delta_y
    :reader delta_y
    :initarg :delta_y
    :type cl:float
    :initform 0.0)
   (aim_offsets_valid
    :reader aim_offsets_valid
    :initarg :aim_offsets_valid
    :type cl:boolean
    :initform cl:nil)
   (a_crosshair_x
    :reader a_crosshair_x
    :initarg :a_crosshair_x
    :type cl:float
    :initform 0.0)
   (a_crosshair_y
    :reader a_crosshair_y
    :initarg :a_crosshair_y
    :type cl:float
    :initform 0.0)
   (a_delta_x
    :reader a_delta_x
    :initarg :a_delta_x
    :type cl:float
    :initform 0.0)
   (a_delta_y
    :reader a_delta_y
    :initarg :a_delta_y
    :type cl:float
    :initform 0.0)
   (b_crosshair_x
    :reader b_crosshair_x
    :initarg :b_crosshair_x
    :type cl:float
    :initform 0.0)
   (b_crosshair_y
    :reader b_crosshair_y
    :initarg :b_crosshair_y
    :type cl:float
    :initform 0.0)
   (b_delta_x
    :reader b_delta_x
    :initarg :b_delta_x
    :type cl:float
    :initform 0.0)
   (b_delta_y
    :reader b_delta_y
    :initarg :b_delta_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass BucketInfo (<BucketInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BucketInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BucketInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cuadc_vision-msg:<BucketInfo> is deprecated: use cuadc_vision-msg:BucketInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:header-val is deprecated.  Use cuadc_vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:count-val is deprecated.  Use cuadc_vision-msg:count instead.")
  (count m))

(cl:ensure-generic-function 'delta_x-val :lambda-list '(m))
(cl:defmethod delta_x-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:delta_x-val is deprecated.  Use cuadc_vision-msg:delta_x instead.")
  (delta_x m))

(cl:ensure-generic-function 'delta_y-val :lambda-list '(m))
(cl:defmethod delta_y-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:delta_y-val is deprecated.  Use cuadc_vision-msg:delta_y instead.")
  (delta_y m))

(cl:ensure-generic-function 'aim_offsets_valid-val :lambda-list '(m))
(cl:defmethod aim_offsets_valid-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:aim_offsets_valid-val is deprecated.  Use cuadc_vision-msg:aim_offsets_valid instead.")
  (aim_offsets_valid m))

(cl:ensure-generic-function 'a_crosshair_x-val :lambda-list '(m))
(cl:defmethod a_crosshair_x-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_crosshair_x-val is deprecated.  Use cuadc_vision-msg:a_crosshair_x instead.")
  (a_crosshair_x m))

(cl:ensure-generic-function 'a_crosshair_y-val :lambda-list '(m))
(cl:defmethod a_crosshair_y-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_crosshair_y-val is deprecated.  Use cuadc_vision-msg:a_crosshair_y instead.")
  (a_crosshair_y m))

(cl:ensure-generic-function 'a_delta_x-val :lambda-list '(m))
(cl:defmethod a_delta_x-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_delta_x-val is deprecated.  Use cuadc_vision-msg:a_delta_x instead.")
  (a_delta_x m))

(cl:ensure-generic-function 'a_delta_y-val :lambda-list '(m))
(cl:defmethod a_delta_y-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_delta_y-val is deprecated.  Use cuadc_vision-msg:a_delta_y instead.")
  (a_delta_y m))

(cl:ensure-generic-function 'b_crosshair_x-val :lambda-list '(m))
(cl:defmethod b_crosshair_x-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_crosshair_x-val is deprecated.  Use cuadc_vision-msg:b_crosshair_x instead.")
  (b_crosshair_x m))

(cl:ensure-generic-function 'b_crosshair_y-val :lambda-list '(m))
(cl:defmethod b_crosshair_y-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_crosshair_y-val is deprecated.  Use cuadc_vision-msg:b_crosshair_y instead.")
  (b_crosshair_y m))

(cl:ensure-generic-function 'b_delta_x-val :lambda-list '(m))
(cl:defmethod b_delta_x-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_delta_x-val is deprecated.  Use cuadc_vision-msg:b_delta_x instead.")
  (b_delta_x m))

(cl:ensure-generic-function 'b_delta_y-val :lambda-list '(m))
(cl:defmethod b_delta_y-val ((m <BucketInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_delta_y-val is deprecated.  Use cuadc_vision-msg:b_delta_y instead.")
  (b_delta_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BucketInfo>) ostream)
  "Serializes a message object of type '<BucketInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'aim_offsets_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_crosshair_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_crosshair_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_delta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_delta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_crosshair_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_crosshair_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_delta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_delta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BucketInfo>) istream)
  "Deserializes a message object of type '<BucketInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'aim_offsets_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_crosshair_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_crosshair_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_delta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_delta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_crosshair_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_crosshair_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_delta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_delta_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BucketInfo>)))
  "Returns string type for a message object of type '<BucketInfo>"
  "cuadc_vision/BucketInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BucketInfo)))
  "Returns string type for a message object of type 'BucketInfo"
  "cuadc_vision/BucketInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BucketInfo>)))
  "Returns md5sum for a message object of type '<BucketInfo>"
  "bed88d5fcfa32d568a9fb5d9f22c5d7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BucketInfo)))
  "Returns md5sum for a message object of type 'BucketInfo"
  "bed88d5fcfa32d568a9fb5d9f22c5d7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BucketInfo>)))
  "Returns full string definition for message of type '<BucketInfo>"
  (cl:format cl:nil "Header header~%int32 count~%float32 delta_x~%float32 delta_y~%bool aim_offsets_valid~%float32 a_crosshair_x~%float32 a_crosshair_y~%float32 a_delta_x~%float32 a_delta_y~%float32 b_crosshair_x~%float32 b_crosshair_y~%float32 b_delta_x~%float32 b_delta_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BucketInfo)))
  "Returns full string definition for message of type 'BucketInfo"
  (cl:format cl:nil "Header header~%int32 count~%float32 delta_x~%float32 delta_y~%bool aim_offsets_valid~%float32 a_crosshair_x~%float32 a_crosshair_y~%float32 a_delta_x~%float32 a_delta_y~%float32 b_crosshair_x~%float32 b_crosshair_y~%float32 b_delta_x~%float32 b_delta_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BucketInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BucketInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'BucketInfo
    (cl:cons ':header (header msg))
    (cl:cons ':count (count msg))
    (cl:cons ':delta_x (delta_x msg))
    (cl:cons ':delta_y (delta_y msg))
    (cl:cons ':aim_offsets_valid (aim_offsets_valid msg))
    (cl:cons ':a_crosshair_x (a_crosshair_x msg))
    (cl:cons ':a_crosshair_y (a_crosshair_y msg))
    (cl:cons ':a_delta_x (a_delta_x msg))
    (cl:cons ':a_delta_y (a_delta_y msg))
    (cl:cons ':b_crosshair_x (b_crosshair_x msg))
    (cl:cons ':b_crosshair_y (b_crosshair_y msg))
    (cl:cons ':b_delta_x (b_delta_x msg))
    (cl:cons ':b_delta_y (b_delta_y msg))
))
