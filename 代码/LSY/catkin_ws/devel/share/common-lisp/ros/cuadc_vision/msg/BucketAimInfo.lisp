; Auto-generated. Do not edit!


(cl:in-package cuadc_vision-msg)


;//! \htmlinclude BucketAimInfo.msg.html

(cl:defclass <BucketAimInfo> (roslisp-msg-protocol:ros-message)
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
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil)
   (bucket_center_x
    :reader bucket_center_x
    :initarg :bucket_center_x
    :type cl:integer
    :initform 0)
   (bucket_center_y
    :reader bucket_center_y
    :initarg :bucket_center_y
    :type cl:integer
    :initform 0)
   (a_aim_x
    :reader a_aim_x
    :initarg :a_aim_x
    :type cl:integer
    :initform 0)
   (a_aim_y
    :reader a_aim_y
    :initarg :a_aim_y
    :type cl:integer
    :initform 0)
   (b_aim_x
    :reader b_aim_x
    :initarg :b_aim_x
    :type cl:integer
    :initform 0)
   (b_aim_y
    :reader b_aim_y
    :initarg :b_aim_y
    :type cl:integer
    :initform 0)
   (a_delta_x_px
    :reader a_delta_x_px
    :initarg :a_delta_x_px
    :type cl:float
    :initform 0.0)
   (a_delta_y_px
    :reader a_delta_y_px
    :initarg :a_delta_y_px
    :type cl:float
    :initform 0.0)
   (b_delta_x_px
    :reader b_delta_x_px
    :initarg :b_delta_x_px
    :type cl:float
    :initform 0.0)
   (b_delta_y_px
    :reader b_delta_y_px
    :initarg :b_delta_y_px
    :type cl:float
    :initform 0.0)
   (a_delta_x_m
    :reader a_delta_x_m
    :initarg :a_delta_x_m
    :type cl:float
    :initform 0.0)
   (a_delta_y_m
    :reader a_delta_y_m
    :initarg :a_delta_y_m
    :type cl:float
    :initform 0.0)
   (b_delta_x_m
    :reader b_delta_x_m
    :initarg :b_delta_x_m
    :type cl:float
    :initform 0.0)
   (b_delta_y_m
    :reader b_delta_y_m
    :initarg :b_delta_y_m
    :type cl:float
    :initform 0.0)
   (a_ned_valid
    :reader a_ned_valid
    :initarg :a_ned_valid
    :type cl:boolean
    :initform cl:nil)
   (a_ned_n
    :reader a_ned_n
    :initarg :a_ned_n
    :type cl:float
    :initform 0.0)
   (a_ned_e
    :reader a_ned_e
    :initarg :a_ned_e
    :type cl:float
    :initform 0.0)
   (a_ned_d
    :reader a_ned_d
    :initarg :a_ned_d
    :type cl:float
    :initform 0.0)
   (b_ned_valid
    :reader b_ned_valid
    :initarg :b_ned_valid
    :type cl:boolean
    :initform cl:nil)
   (b_ned_n
    :reader b_ned_n
    :initarg :b_ned_n
    :type cl:float
    :initform 0.0)
   (b_ned_e
    :reader b_ned_e
    :initarg :b_ned_e
    :type cl:float
    :initform 0.0)
   (b_ned_d
    :reader b_ned_d
    :initarg :b_ned_d
    :type cl:float
    :initform 0.0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (source_class
    :reader source_class
    :initarg :source_class
    :type cl:string
    :initform ""))
)

(cl:defclass BucketAimInfo (<BucketAimInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BucketAimInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BucketAimInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cuadc_vision-msg:<BucketAimInfo> is deprecated: use cuadc_vision-msg:BucketAimInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:header-val is deprecated.  Use cuadc_vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:count-val is deprecated.  Use cuadc_vision-msg:count instead.")
  (count m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:valid-val is deprecated.  Use cuadc_vision-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'bucket_center_x-val :lambda-list '(m))
(cl:defmethod bucket_center_x-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:bucket_center_x-val is deprecated.  Use cuadc_vision-msg:bucket_center_x instead.")
  (bucket_center_x m))

(cl:ensure-generic-function 'bucket_center_y-val :lambda-list '(m))
(cl:defmethod bucket_center_y-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:bucket_center_y-val is deprecated.  Use cuadc_vision-msg:bucket_center_y instead.")
  (bucket_center_y m))

(cl:ensure-generic-function 'a_aim_x-val :lambda-list '(m))
(cl:defmethod a_aim_x-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_aim_x-val is deprecated.  Use cuadc_vision-msg:a_aim_x instead.")
  (a_aim_x m))

(cl:ensure-generic-function 'a_aim_y-val :lambda-list '(m))
(cl:defmethod a_aim_y-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_aim_y-val is deprecated.  Use cuadc_vision-msg:a_aim_y instead.")
  (a_aim_y m))

(cl:ensure-generic-function 'b_aim_x-val :lambda-list '(m))
(cl:defmethod b_aim_x-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_aim_x-val is deprecated.  Use cuadc_vision-msg:b_aim_x instead.")
  (b_aim_x m))

(cl:ensure-generic-function 'b_aim_y-val :lambda-list '(m))
(cl:defmethod b_aim_y-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_aim_y-val is deprecated.  Use cuadc_vision-msg:b_aim_y instead.")
  (b_aim_y m))

(cl:ensure-generic-function 'a_delta_x_px-val :lambda-list '(m))
(cl:defmethod a_delta_x_px-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_delta_x_px-val is deprecated.  Use cuadc_vision-msg:a_delta_x_px instead.")
  (a_delta_x_px m))

(cl:ensure-generic-function 'a_delta_y_px-val :lambda-list '(m))
(cl:defmethod a_delta_y_px-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_delta_y_px-val is deprecated.  Use cuadc_vision-msg:a_delta_y_px instead.")
  (a_delta_y_px m))

(cl:ensure-generic-function 'b_delta_x_px-val :lambda-list '(m))
(cl:defmethod b_delta_x_px-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_delta_x_px-val is deprecated.  Use cuadc_vision-msg:b_delta_x_px instead.")
  (b_delta_x_px m))

(cl:ensure-generic-function 'b_delta_y_px-val :lambda-list '(m))
(cl:defmethod b_delta_y_px-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_delta_y_px-val is deprecated.  Use cuadc_vision-msg:b_delta_y_px instead.")
  (b_delta_y_px m))

(cl:ensure-generic-function 'a_delta_x_m-val :lambda-list '(m))
(cl:defmethod a_delta_x_m-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_delta_x_m-val is deprecated.  Use cuadc_vision-msg:a_delta_x_m instead.")
  (a_delta_x_m m))

(cl:ensure-generic-function 'a_delta_y_m-val :lambda-list '(m))
(cl:defmethod a_delta_y_m-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_delta_y_m-val is deprecated.  Use cuadc_vision-msg:a_delta_y_m instead.")
  (a_delta_y_m m))

(cl:ensure-generic-function 'b_delta_x_m-val :lambda-list '(m))
(cl:defmethod b_delta_x_m-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_delta_x_m-val is deprecated.  Use cuadc_vision-msg:b_delta_x_m instead.")
  (b_delta_x_m m))

(cl:ensure-generic-function 'b_delta_y_m-val :lambda-list '(m))
(cl:defmethod b_delta_y_m-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_delta_y_m-val is deprecated.  Use cuadc_vision-msg:b_delta_y_m instead.")
  (b_delta_y_m m))

(cl:ensure-generic-function 'a_ned_valid-val :lambda-list '(m))
(cl:defmethod a_ned_valid-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_ned_valid-val is deprecated.  Use cuadc_vision-msg:a_ned_valid instead.")
  (a_ned_valid m))

(cl:ensure-generic-function 'a_ned_n-val :lambda-list '(m))
(cl:defmethod a_ned_n-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_ned_n-val is deprecated.  Use cuadc_vision-msg:a_ned_n instead.")
  (a_ned_n m))

(cl:ensure-generic-function 'a_ned_e-val :lambda-list '(m))
(cl:defmethod a_ned_e-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_ned_e-val is deprecated.  Use cuadc_vision-msg:a_ned_e instead.")
  (a_ned_e m))

(cl:ensure-generic-function 'a_ned_d-val :lambda-list '(m))
(cl:defmethod a_ned_d-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:a_ned_d-val is deprecated.  Use cuadc_vision-msg:a_ned_d instead.")
  (a_ned_d m))

(cl:ensure-generic-function 'b_ned_valid-val :lambda-list '(m))
(cl:defmethod b_ned_valid-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_ned_valid-val is deprecated.  Use cuadc_vision-msg:b_ned_valid instead.")
  (b_ned_valid m))

(cl:ensure-generic-function 'b_ned_n-val :lambda-list '(m))
(cl:defmethod b_ned_n-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_ned_n-val is deprecated.  Use cuadc_vision-msg:b_ned_n instead.")
  (b_ned_n m))

(cl:ensure-generic-function 'b_ned_e-val :lambda-list '(m))
(cl:defmethod b_ned_e-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_ned_e-val is deprecated.  Use cuadc_vision-msg:b_ned_e instead.")
  (b_ned_e m))

(cl:ensure-generic-function 'b_ned_d-val :lambda-list '(m))
(cl:defmethod b_ned_d-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:b_ned_d-val is deprecated.  Use cuadc_vision-msg:b_ned_d instead.")
  (b_ned_d m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:confidence-val is deprecated.  Use cuadc_vision-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'source_class-val :lambda-list '(m))
(cl:defmethod source_class-val ((m <BucketAimInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:source_class-val is deprecated.  Use cuadc_vision-msg:source_class instead.")
  (source_class m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BucketAimInfo>) ostream)
  "Serializes a message object of type '<BucketAimInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'bucket_center_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bucket_center_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a_aim_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a_aim_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b_aim_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b_aim_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_delta_x_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_delta_y_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_delta_x_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_delta_y_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_delta_x_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_delta_y_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_delta_x_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_delta_y_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'a_ned_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_ned_n))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_ned_e))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a_ned_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b_ned_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_ned_n))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_ned_e))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b_ned_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source_class))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source_class))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BucketAimInfo>) istream)
  "Deserializes a message object of type '<BucketAimInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bucket_center_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bucket_center_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a_aim_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a_aim_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b_aim_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b_aim_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_delta_x_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_delta_y_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_delta_x_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_delta_y_px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_delta_x_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_delta_y_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_delta_x_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_delta_y_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'a_ned_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_ned_n) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_ned_e) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_ned_d) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'b_ned_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_ned_n) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_ned_e) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b_ned_d) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source_class) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source_class) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BucketAimInfo>)))
  "Returns string type for a message object of type '<BucketAimInfo>"
  "cuadc_vision/BucketAimInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BucketAimInfo)))
  "Returns string type for a message object of type 'BucketAimInfo"
  "cuadc_vision/BucketAimInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BucketAimInfo>)))
  "Returns md5sum for a message object of type '<BucketAimInfo>"
  "781270f5d3447ff4d4ce3194c04670ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BucketAimInfo)))
  "Returns md5sum for a message object of type 'BucketAimInfo"
  "781270f5d3447ff4d4ce3194c04670ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BucketAimInfo>)))
  "Returns full string definition for message of type '<BucketAimInfo>"
  (cl:format cl:nil "# Stable bucket center and physical A/B dropper aim points.~%Header header~%int32 count~%~%# True when detection, depth, intrinsics, pixel aim points and metric deltas are valid.~%# NED validity is reported independently because local_pose may be unavailable.~%bool valid~%~%int32 bucket_center_x~%int32 bucket_center_y~%~%int32 a_aim_x~%int32 a_aim_y~%int32 b_aim_x~%int32 b_aim_y~%~%# Delta convention: delta = aim_point - image_center.~%# Pixel deltas use image axes (x right, y down).~%float32 a_delta_x_px~%float32 a_delta_y_px~%float32 b_delta_x_px~%float32 b_delta_y_px~%~%# Metric deltas are estimated from the current target depth and camera intrinsics.~%float32 a_delta_x_m~%float32 a_delta_y_m~%float32 b_delta_x_m~%float32 b_delta_y_m~%~%# Absolute target positions in the MAVROS local NED frame.~%bool a_ned_valid~%float32 a_ned_n~%float32 a_ned_e~%float32 a_ned_d~%~%bool b_ned_valid~%float32 b_ned_n~%float32 b_ned_e~%float32 b_ned_d~%~%float32 confidence~%string source_class~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BucketAimInfo)))
  "Returns full string definition for message of type 'BucketAimInfo"
  (cl:format cl:nil "# Stable bucket center and physical A/B dropper aim points.~%Header header~%int32 count~%~%# True when detection, depth, intrinsics, pixel aim points and metric deltas are valid.~%# NED validity is reported independently because local_pose may be unavailable.~%bool valid~%~%int32 bucket_center_x~%int32 bucket_center_y~%~%int32 a_aim_x~%int32 a_aim_y~%int32 b_aim_x~%int32 b_aim_y~%~%# Delta convention: delta = aim_point - image_center.~%# Pixel deltas use image axes (x right, y down).~%float32 a_delta_x_px~%float32 a_delta_y_px~%float32 b_delta_x_px~%float32 b_delta_y_px~%~%# Metric deltas are estimated from the current target depth and camera intrinsics.~%float32 a_delta_x_m~%float32 a_delta_y_m~%float32 b_delta_x_m~%float32 b_delta_y_m~%~%# Absolute target positions in the MAVROS local NED frame.~%bool a_ned_valid~%float32 a_ned_n~%float32 a_ned_e~%float32 a_ned_d~%~%bool b_ned_valid~%float32 b_ned_n~%float32 b_ned_e~%float32 b_ned_d~%~%float32 confidence~%string source_class~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BucketAimInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
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
     4
     4
     4
     4
     4
     4
     1
     4
     4
     4
     1
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'source_class))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BucketAimInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'BucketAimInfo
    (cl:cons ':header (header msg))
    (cl:cons ':count (count msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':bucket_center_x (bucket_center_x msg))
    (cl:cons ':bucket_center_y (bucket_center_y msg))
    (cl:cons ':a_aim_x (a_aim_x msg))
    (cl:cons ':a_aim_y (a_aim_y msg))
    (cl:cons ':b_aim_x (b_aim_x msg))
    (cl:cons ':b_aim_y (b_aim_y msg))
    (cl:cons ':a_delta_x_px (a_delta_x_px msg))
    (cl:cons ':a_delta_y_px (a_delta_y_px msg))
    (cl:cons ':b_delta_x_px (b_delta_x_px msg))
    (cl:cons ':b_delta_y_px (b_delta_y_px msg))
    (cl:cons ':a_delta_x_m (a_delta_x_m msg))
    (cl:cons ':a_delta_y_m (a_delta_y_m msg))
    (cl:cons ':b_delta_x_m (b_delta_x_m msg))
    (cl:cons ':b_delta_y_m (b_delta_y_m msg))
    (cl:cons ':a_ned_valid (a_ned_valid msg))
    (cl:cons ':a_ned_n (a_ned_n msg))
    (cl:cons ':a_ned_e (a_ned_e msg))
    (cl:cons ':a_ned_d (a_ned_d msg))
    (cl:cons ':b_ned_valid (b_ned_valid msg))
    (cl:cons ':b_ned_n (b_ned_n msg))
    (cl:cons ':b_ned_e (b_ned_e msg))
    (cl:cons ':b_ned_d (b_ned_d msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':source_class (source_class msg))
))
