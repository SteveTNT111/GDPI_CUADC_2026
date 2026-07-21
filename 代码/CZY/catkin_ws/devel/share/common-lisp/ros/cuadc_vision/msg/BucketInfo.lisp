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
  "6e7635ad7a87ad436d861b2afc3bc251")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BucketInfo)))
  "Returns md5sum for a message object of type 'BucketInfo"
  "6e7635ad7a87ad436d861b2afc3bc251")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BucketInfo>)))
  "Returns full string definition for message of type '<BucketInfo>"
  (cl:format cl:nil "Header header~%int32 count~%float32 delta_x~%float32 delta_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BucketInfo)))
  "Returns full string definition for message of type 'BucketInfo"
  (cl:format cl:nil "Header header~%int32 count~%float32 delta_x~%float32 delta_y~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BucketInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
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
))
