; Auto-generated. Do not edit!


(cl:in-package cuadc_vision-msg)


;//! \htmlinclude MissionStatus.msg.html

(cl:defclass <MissionStatus> (roslisp-msg-protocol:ros-message)
  ((ammo_a
    :reader ammo_a
    :initarg :ammo_a
    :type cl:fixnum
    :initform 0)
   (ammo_b
    :reader ammo_b
    :initarg :ammo_b
    :type cl:fixnum
    :initform 0)
   (aiming
    :reader aiming
    :initarg :aiming
    :type cl:boolean
    :initform cl:nil)
   (last_drop
    :reader last_drop
    :initarg :last_drop
    :type cl:string
    :initform ""))
)

(cl:defclass MissionStatus (<MissionStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MissionStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MissionStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cuadc_vision-msg:<MissionStatus> is deprecated: use cuadc_vision-msg:MissionStatus instead.")))

(cl:ensure-generic-function 'ammo_a-val :lambda-list '(m))
(cl:defmethod ammo_a-val ((m <MissionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:ammo_a-val is deprecated.  Use cuadc_vision-msg:ammo_a instead.")
  (ammo_a m))

(cl:ensure-generic-function 'ammo_b-val :lambda-list '(m))
(cl:defmethod ammo_b-val ((m <MissionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:ammo_b-val is deprecated.  Use cuadc_vision-msg:ammo_b instead.")
  (ammo_b m))

(cl:ensure-generic-function 'aiming-val :lambda-list '(m))
(cl:defmethod aiming-val ((m <MissionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:aiming-val is deprecated.  Use cuadc_vision-msg:aiming instead.")
  (aiming m))

(cl:ensure-generic-function 'last_drop-val :lambda-list '(m))
(cl:defmethod last_drop-val ((m <MissionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cuadc_vision-msg:last_drop-val is deprecated.  Use cuadc_vision-msg:last_drop instead.")
  (last_drop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MissionStatus>) ostream)
  "Serializes a message object of type '<MissionStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ammo_a)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ammo_b)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'aiming) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'last_drop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'last_drop))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MissionStatus>) istream)
  "Deserializes a message object of type '<MissionStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ammo_a)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ammo_b)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'aiming) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'last_drop) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'last_drop) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MissionStatus>)))
  "Returns string type for a message object of type '<MissionStatus>"
  "cuadc_vision/MissionStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MissionStatus)))
  "Returns string type for a message object of type 'MissionStatus"
  "cuadc_vision/MissionStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MissionStatus>)))
  "Returns md5sum for a message object of type '<MissionStatus>"
  "42a0745a697dd6adddaac7f2c09f5f59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MissionStatus)))
  "Returns md5sum for a message object of type 'MissionStatus"
  "42a0745a697dd6adddaac7f2c09f5f59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MissionStatus>)))
  "Returns full string definition for message of type '<MissionStatus>"
  (cl:format cl:nil "# MissionStatus.msg — main.py → detector_node 任务状态同步~%#~%# 用于 detector_node 在画面上显示弹药、瞄准、抛投等任务状态信息。~%~%uint8 ammo_a                # 前抛投器 (A) 剩余弹药数，0 表示无/未挂载~%uint8 ammo_b                # 后抛投器 (B) 剩余弹药数，0 表示无/未挂载~%bool aiming                 # 飞控处于 GUIDED 模式且正在执行对准任务~%string last_drop            # 最近一次抛投的抛投器编号 (\"A\" / \"B\" / \"\")~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MissionStatus)))
  "Returns full string definition for message of type 'MissionStatus"
  (cl:format cl:nil "# MissionStatus.msg — main.py → detector_node 任务状态同步~%#~%# 用于 detector_node 在画面上显示弹药、瞄准、抛投等任务状态信息。~%~%uint8 ammo_a                # 前抛投器 (A) 剩余弹药数，0 表示无/未挂载~%uint8 ammo_b                # 后抛投器 (B) 剩余弹药数，0 表示无/未挂载~%bool aiming                 # 飞控处于 GUIDED 模式且正在执行对准任务~%string last_drop            # 最近一次抛投的抛投器编号 (\"A\" / \"B\" / \"\")~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MissionStatus>))
  (cl:+ 0
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'last_drop))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MissionStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'MissionStatus
    (cl:cons ':ammo_a (ammo_a msg))
    (cl:cons ':ammo_b (ammo_b msg))
    (cl:cons ':aiming (aiming msg))
    (cl:cons ':last_drop (last_drop msg))
))
