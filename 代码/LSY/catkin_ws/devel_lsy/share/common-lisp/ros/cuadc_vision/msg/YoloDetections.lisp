; Auto-generated. Do not edit!


(cl:in-package cuadc_vision-msg)


;//! \htmlinclude YoloDetections.msg.html

(cl:defclass <YoloDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector cuadc_vision-msg:YoloDetection)
   :initform (cl:make-array 0 :element-type 'cuadc_vision-msg:YoloDetection :initial-element (cl:make-instance 'cuadc_vision-msg:YoloDetection))))
)

(cl:defclass YoloDetections (<YoloDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YoloDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YoloDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cuadc_vision-msg:<YoloDetections> is deprecated: use cuadc_vision-msg:YoloDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <YoloDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:header-val is deprecated.  Use cuadc_vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <YoloDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:detections-val is deprecated.  Use cuadc_vision-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YoloDetections>) ostream)
  "Serializes a message object of type '<YoloDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YoloDetections>) istream)
  "Deserializes a message object of type '<YoloDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cuadc_vision-msg:YoloDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YoloDetections>)))
  "Returns string type for a message object of type '<YoloDetections>"
  "cuadc_vision/YoloDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YoloDetections)))
  "Returns string type for a message object of type 'YoloDetections"
  "cuadc_vision/YoloDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YoloDetections>)))
  "Returns md5sum for a message object of type '<YoloDetections>"
  "a930238f1ca26e45c4d2a892c5bf7982")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YoloDetections)))
  "Returns md5sum for a message object of type 'YoloDetections"
  "a930238f1ca26e45c4d2a892c5bf7982")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YoloDetections>)))
  "Returns full string definition for message of type '<YoloDetections>"
  (cl:format cl:nil "Header header~%YoloDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: cuadc_vision/YoloDetection~%Header header~%bool detected~%int32 class_id~%string class_name~%float32 confidence~%int32 x_min~%int32 y_min~%int32 x_max~%int32 y_max~%int32 center_x~%int32 center_y~%float32 depth_m~%bool position_valid~%float32 camera_x_m~%float32 camera_y_m~%float32 camera_z_m~%float32 distance_m~%float32 bbox_width_m~%float32 bbox_height_m~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YoloDetections)))
  "Returns full string definition for message of type 'YoloDetections"
  (cl:format cl:nil "Header header~%YoloDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: cuadc_vision/YoloDetection~%Header header~%bool detected~%int32 class_id~%string class_name~%float32 confidence~%int32 x_min~%int32 y_min~%int32 x_max~%int32 y_max~%int32 center_x~%int32 center_y~%float32 depth_m~%bool position_valid~%float32 camera_x_m~%float32 camera_y_m~%float32 camera_z_m~%float32 distance_m~%float32 bbox_width_m~%float32 bbox_height_m~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YoloDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YoloDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'YoloDetections
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
