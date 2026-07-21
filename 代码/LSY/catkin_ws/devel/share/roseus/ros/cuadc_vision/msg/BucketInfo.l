;; Auto-generated. Do not edit!


(when (boundp 'cuadc_vision::BucketInfo)
  (if (not (find-package "CUADC_VISION"))
    (make-package "CUADC_VISION"))
  (shadow 'BucketInfo (find-package "CUADC_VISION")))
(unless (find-package "CUADC_VISION::BUCKETINFO")
  (make-package "CUADC_VISION::BUCKETINFO"))

(in-package "ROS")
;;//! \htmlinclude BucketInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cuadc_vision::BucketInfo
  :super ros::object
  :slots (_header _count _delta_x _delta_y _aim_offsets_valid _a_crosshair_x _a_crosshair_y _a_delta_x _a_delta_y _b_crosshair_x _b_crosshair_y _b_delta_x _b_delta_y ))

(defmethod cuadc_vision::BucketInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:count __count) 0)
    ((:delta_x __delta_x) 0.0)
    ((:delta_y __delta_y) 0.0)
    ((:aim_offsets_valid __aim_offsets_valid) nil)
    ((:a_crosshair_x __a_crosshair_x) 0.0)
    ((:a_crosshair_y __a_crosshair_y) 0.0)
    ((:a_delta_x __a_delta_x) 0.0)
    ((:a_delta_y __a_delta_y) 0.0)
    ((:b_crosshair_x __b_crosshair_x) 0.0)
    ((:b_crosshair_y __b_crosshair_y) 0.0)
    ((:b_delta_x __b_delta_x) 0.0)
    ((:b_delta_y __b_delta_y) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _count (round __count))
   (setq _delta_x (float __delta_x))
   (setq _delta_y (float __delta_y))
   (setq _aim_offsets_valid __aim_offsets_valid)
   (setq _a_crosshair_x (float __a_crosshair_x))
   (setq _a_crosshair_y (float __a_crosshair_y))
   (setq _a_delta_x (float __a_delta_x))
   (setq _a_delta_y (float __a_delta_y))
   (setq _b_crosshair_x (float __b_crosshair_x))
   (setq _b_crosshair_y (float __b_crosshair_y))
   (setq _b_delta_x (float __b_delta_x))
   (setq _b_delta_y (float __b_delta_y))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:delta_x
   (&optional __delta_x)
   (if __delta_x (setq _delta_x __delta_x)) _delta_x)
  (:delta_y
   (&optional __delta_y)
   (if __delta_y (setq _delta_y __delta_y)) _delta_y)
  (:aim_offsets_valid
   (&optional (__aim_offsets_valid :null))
   (if (not (eq __aim_offsets_valid :null)) (setq _aim_offsets_valid __aim_offsets_valid)) _aim_offsets_valid)
  (:a_crosshair_x
   (&optional __a_crosshair_x)
   (if __a_crosshair_x (setq _a_crosshair_x __a_crosshair_x)) _a_crosshair_x)
  (:a_crosshair_y
   (&optional __a_crosshair_y)
   (if __a_crosshair_y (setq _a_crosshair_y __a_crosshair_y)) _a_crosshair_y)
  (:a_delta_x
   (&optional __a_delta_x)
   (if __a_delta_x (setq _a_delta_x __a_delta_x)) _a_delta_x)
  (:a_delta_y
   (&optional __a_delta_y)
   (if __a_delta_y (setq _a_delta_y __a_delta_y)) _a_delta_y)
  (:b_crosshair_x
   (&optional __b_crosshair_x)
   (if __b_crosshair_x (setq _b_crosshair_x __b_crosshair_x)) _b_crosshair_x)
  (:b_crosshair_y
   (&optional __b_crosshair_y)
   (if __b_crosshair_y (setq _b_crosshair_y __b_crosshair_y)) _b_crosshair_y)
  (:b_delta_x
   (&optional __b_delta_x)
   (if __b_delta_x (setq _b_delta_x __b_delta_x)) _b_delta_x)
  (:b_delta_y
   (&optional __b_delta_y)
   (if __b_delta_y (setq _b_delta_y __b_delta_y)) _b_delta_y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _count
    4
    ;; float32 _delta_x
    4
    ;; float32 _delta_y
    4
    ;; bool _aim_offsets_valid
    1
    ;; float32 _a_crosshair_x
    4
    ;; float32 _a_crosshair_y
    4
    ;; float32 _a_delta_x
    4
    ;; float32 _a_delta_y
    4
    ;; float32 _b_crosshair_x
    4
    ;; float32 _b_crosshair_y
    4
    ;; float32 _b_delta_x
    4
    ;; float32 _b_delta_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _count
       (write-long _count s)
     ;; float32 _delta_x
       (sys::poke _delta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_y
       (sys::poke _delta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _aim_offsets_valid
       (if _aim_offsets_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _a_crosshair_x
       (sys::poke _a_crosshair_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_crosshair_y
       (sys::poke _a_crosshair_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_delta_x
       (sys::poke _a_delta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_delta_y
       (sys::poke _a_delta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_crosshair_x
       (sys::poke _b_crosshair_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_crosshair_y
       (sys::poke _b_crosshair_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_delta_x
       (sys::poke _b_delta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_delta_y
       (sys::poke _b_delta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _delta_x
     (setq _delta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_y
     (setq _delta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _aim_offsets_valid
     (setq _aim_offsets_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _a_crosshair_x
     (setq _a_crosshair_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_crosshair_y
     (setq _a_crosshair_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_delta_x
     (setq _a_delta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_delta_y
     (setq _a_delta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_crosshair_x
     (setq _b_crosshair_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_crosshair_y
     (setq _b_crosshair_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_delta_x
     (setq _b_delta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_delta_y
     (setq _b_delta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cuadc_vision::BucketInfo :md5sum-) "bed88d5fcfa32d568a9fb5d9f22c5d7b")
(setf (get cuadc_vision::BucketInfo :datatype-) "cuadc_vision/BucketInfo")
(setf (get cuadc_vision::BucketInfo :definition-)
      "Header header
int32 count
float32 delta_x
float32 delta_y
bool aim_offsets_valid
float32 a_crosshair_x
float32 a_crosshair_y
float32 a_delta_x
float32 a_delta_y
float32 b_crosshair_x
float32 b_crosshair_y
float32 b_delta_x
float32 b_delta_y

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

")



(provide :cuadc_vision/BucketInfo "bed88d5fcfa32d568a9fb5d9f22c5d7b")


