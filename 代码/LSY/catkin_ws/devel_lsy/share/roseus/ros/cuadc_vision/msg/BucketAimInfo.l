;; Auto-generated. Do not edit!


(when (boundp 'cuadc_vision::BucketAimInfo)
  (if (not (find-package "CUADC_VISION"))
    (make-package "CUADC_VISION"))
  (shadow 'BucketAimInfo (find-package "CUADC_VISION")))
(unless (find-package "CUADC_VISION::BUCKETAIMINFO")
  (make-package "CUADC_VISION::BUCKETAIMINFO"))

(in-package "ROS")
;;//! \htmlinclude BucketAimInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cuadc_vision::BucketAimInfo
  :super ros::object
  :slots (_header _count _valid _bucket_center_x _bucket_center_y _a_aim_x _a_aim_y _b_aim_x _b_aim_y _a_delta_x_px _a_delta_y_px _b_delta_x_px _b_delta_y_px _a_delta_x_m _a_delta_y_m _b_delta_x_m _b_delta_y_m _a_ned_valid _a_ned_n _a_ned_e _a_ned_d _b_ned_valid _b_ned_n _b_ned_e _b_ned_d _confidence _source_class ))

(defmethod cuadc_vision::BucketAimInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:count __count) 0)
    ((:valid __valid) nil)
    ((:bucket_center_x __bucket_center_x) 0)
    ((:bucket_center_y __bucket_center_y) 0)
    ((:a_aim_x __a_aim_x) 0)
    ((:a_aim_y __a_aim_y) 0)
    ((:b_aim_x __b_aim_x) 0)
    ((:b_aim_y __b_aim_y) 0)
    ((:a_delta_x_px __a_delta_x_px) 0.0)
    ((:a_delta_y_px __a_delta_y_px) 0.0)
    ((:b_delta_x_px __b_delta_x_px) 0.0)
    ((:b_delta_y_px __b_delta_y_px) 0.0)
    ((:a_delta_x_m __a_delta_x_m) 0.0)
    ((:a_delta_y_m __a_delta_y_m) 0.0)
    ((:b_delta_x_m __b_delta_x_m) 0.0)
    ((:b_delta_y_m __b_delta_y_m) 0.0)
    ((:a_ned_valid __a_ned_valid) nil)
    ((:a_ned_n __a_ned_n) 0.0)
    ((:a_ned_e __a_ned_e) 0.0)
    ((:a_ned_d __a_ned_d) 0.0)
    ((:b_ned_valid __b_ned_valid) nil)
    ((:b_ned_n __b_ned_n) 0.0)
    ((:b_ned_e __b_ned_e) 0.0)
    ((:b_ned_d __b_ned_d) 0.0)
    ((:confidence __confidence) 0.0)
    ((:source_class __source_class) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _count (round __count))
   (setq _valid __valid)
   (setq _bucket_center_x (round __bucket_center_x))
   (setq _bucket_center_y (round __bucket_center_y))
   (setq _a_aim_x (round __a_aim_x))
   (setq _a_aim_y (round __a_aim_y))
   (setq _b_aim_x (round __b_aim_x))
   (setq _b_aim_y (round __b_aim_y))
   (setq _a_delta_x_px (float __a_delta_x_px))
   (setq _a_delta_y_px (float __a_delta_y_px))
   (setq _b_delta_x_px (float __b_delta_x_px))
   (setq _b_delta_y_px (float __b_delta_y_px))
   (setq _a_delta_x_m (float __a_delta_x_m))
   (setq _a_delta_y_m (float __a_delta_y_m))
   (setq _b_delta_x_m (float __b_delta_x_m))
   (setq _b_delta_y_m (float __b_delta_y_m))
   (setq _a_ned_valid __a_ned_valid)
   (setq _a_ned_n (float __a_ned_n))
   (setq _a_ned_e (float __a_ned_e))
   (setq _a_ned_d (float __a_ned_d))
   (setq _b_ned_valid __b_ned_valid)
   (setq _b_ned_n (float __b_ned_n))
   (setq _b_ned_e (float __b_ned_e))
   (setq _b_ned_d (float __b_ned_d))
   (setq _confidence (float __confidence))
   (setq _source_class (string __source_class))
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
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:bucket_center_x
   (&optional __bucket_center_x)
   (if __bucket_center_x (setq _bucket_center_x __bucket_center_x)) _bucket_center_x)
  (:bucket_center_y
   (&optional __bucket_center_y)
   (if __bucket_center_y (setq _bucket_center_y __bucket_center_y)) _bucket_center_y)
  (:a_aim_x
   (&optional __a_aim_x)
   (if __a_aim_x (setq _a_aim_x __a_aim_x)) _a_aim_x)
  (:a_aim_y
   (&optional __a_aim_y)
   (if __a_aim_y (setq _a_aim_y __a_aim_y)) _a_aim_y)
  (:b_aim_x
   (&optional __b_aim_x)
   (if __b_aim_x (setq _b_aim_x __b_aim_x)) _b_aim_x)
  (:b_aim_y
   (&optional __b_aim_y)
   (if __b_aim_y (setq _b_aim_y __b_aim_y)) _b_aim_y)
  (:a_delta_x_px
   (&optional __a_delta_x_px)
   (if __a_delta_x_px (setq _a_delta_x_px __a_delta_x_px)) _a_delta_x_px)
  (:a_delta_y_px
   (&optional __a_delta_y_px)
   (if __a_delta_y_px (setq _a_delta_y_px __a_delta_y_px)) _a_delta_y_px)
  (:b_delta_x_px
   (&optional __b_delta_x_px)
   (if __b_delta_x_px (setq _b_delta_x_px __b_delta_x_px)) _b_delta_x_px)
  (:b_delta_y_px
   (&optional __b_delta_y_px)
   (if __b_delta_y_px (setq _b_delta_y_px __b_delta_y_px)) _b_delta_y_px)
  (:a_delta_x_m
   (&optional __a_delta_x_m)
   (if __a_delta_x_m (setq _a_delta_x_m __a_delta_x_m)) _a_delta_x_m)
  (:a_delta_y_m
   (&optional __a_delta_y_m)
   (if __a_delta_y_m (setq _a_delta_y_m __a_delta_y_m)) _a_delta_y_m)
  (:b_delta_x_m
   (&optional __b_delta_x_m)
   (if __b_delta_x_m (setq _b_delta_x_m __b_delta_x_m)) _b_delta_x_m)
  (:b_delta_y_m
   (&optional __b_delta_y_m)
   (if __b_delta_y_m (setq _b_delta_y_m __b_delta_y_m)) _b_delta_y_m)
  (:a_ned_valid
   (&optional (__a_ned_valid :null))
   (if (not (eq __a_ned_valid :null)) (setq _a_ned_valid __a_ned_valid)) _a_ned_valid)
  (:a_ned_n
   (&optional __a_ned_n)
   (if __a_ned_n (setq _a_ned_n __a_ned_n)) _a_ned_n)
  (:a_ned_e
   (&optional __a_ned_e)
   (if __a_ned_e (setq _a_ned_e __a_ned_e)) _a_ned_e)
  (:a_ned_d
   (&optional __a_ned_d)
   (if __a_ned_d (setq _a_ned_d __a_ned_d)) _a_ned_d)
  (:b_ned_valid
   (&optional (__b_ned_valid :null))
   (if (not (eq __b_ned_valid :null)) (setq _b_ned_valid __b_ned_valid)) _b_ned_valid)
  (:b_ned_n
   (&optional __b_ned_n)
   (if __b_ned_n (setq _b_ned_n __b_ned_n)) _b_ned_n)
  (:b_ned_e
   (&optional __b_ned_e)
   (if __b_ned_e (setq _b_ned_e __b_ned_e)) _b_ned_e)
  (:b_ned_d
   (&optional __b_ned_d)
   (if __b_ned_d (setq _b_ned_d __b_ned_d)) _b_ned_d)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:source_class
   (&optional __source_class)
   (if __source_class (setq _source_class __source_class)) _source_class)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _count
    4
    ;; bool _valid
    1
    ;; int32 _bucket_center_x
    4
    ;; int32 _bucket_center_y
    4
    ;; int32 _a_aim_x
    4
    ;; int32 _a_aim_y
    4
    ;; int32 _b_aim_x
    4
    ;; int32 _b_aim_y
    4
    ;; float32 _a_delta_x_px
    4
    ;; float32 _a_delta_y_px
    4
    ;; float32 _b_delta_x_px
    4
    ;; float32 _b_delta_y_px
    4
    ;; float32 _a_delta_x_m
    4
    ;; float32 _a_delta_y_m
    4
    ;; float32 _b_delta_x_m
    4
    ;; float32 _b_delta_y_m
    4
    ;; bool _a_ned_valid
    1
    ;; float32 _a_ned_n
    4
    ;; float32 _a_ned_e
    4
    ;; float32 _a_ned_d
    4
    ;; bool _b_ned_valid
    1
    ;; float32 _b_ned_n
    4
    ;; float32 _b_ned_e
    4
    ;; float32 _b_ned_d
    4
    ;; float32 _confidence
    4
    ;; string _source_class
    4 (length _source_class)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _count
       (write-long _count s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; int32 _bucket_center_x
       (write-long _bucket_center_x s)
     ;; int32 _bucket_center_y
       (write-long _bucket_center_y s)
     ;; int32 _a_aim_x
       (write-long _a_aim_x s)
     ;; int32 _a_aim_y
       (write-long _a_aim_y s)
     ;; int32 _b_aim_x
       (write-long _b_aim_x s)
     ;; int32 _b_aim_y
       (write-long _b_aim_y s)
     ;; float32 _a_delta_x_px
       (sys::poke _a_delta_x_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_delta_y_px
       (sys::poke _a_delta_y_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_delta_x_px
       (sys::poke _b_delta_x_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_delta_y_px
       (sys::poke _b_delta_y_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_delta_x_m
       (sys::poke _a_delta_x_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_delta_y_m
       (sys::poke _a_delta_y_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_delta_x_m
       (sys::poke _b_delta_x_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_delta_y_m
       (sys::poke _b_delta_y_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _a_ned_valid
       (if _a_ned_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _a_ned_n
       (sys::poke _a_ned_n (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_ned_e
       (sys::poke _a_ned_e (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a_ned_d
       (sys::poke _a_ned_d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _b_ned_valid
       (if _b_ned_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _b_ned_n
       (sys::poke _b_ned_n (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_ned_e
       (sys::poke _b_ned_e (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_ned_d
       (sys::poke _b_ned_d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _source_class
       (write-long (length _source_class) s) (princ _source_class s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _bucket_center_x
     (setq _bucket_center_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bucket_center_y
     (setq _bucket_center_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _a_aim_x
     (setq _a_aim_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _a_aim_y
     (setq _a_aim_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _b_aim_x
     (setq _b_aim_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _b_aim_y
     (setq _b_aim_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _a_delta_x_px
     (setq _a_delta_x_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_delta_y_px
     (setq _a_delta_y_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_delta_x_px
     (setq _b_delta_x_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_delta_y_px
     (setq _b_delta_y_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_delta_x_m
     (setq _a_delta_x_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_delta_y_m
     (setq _a_delta_y_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_delta_x_m
     (setq _b_delta_x_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_delta_y_m
     (setq _b_delta_y_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _a_ned_valid
     (setq _a_ned_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _a_ned_n
     (setq _a_ned_n (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_ned_e
     (setq _a_ned_e (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a_ned_d
     (setq _a_ned_d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _b_ned_valid
     (setq _b_ned_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _b_ned_n
     (setq _b_ned_n (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_ned_e
     (setq _b_ned_e (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_ned_d
     (setq _b_ned_d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _source_class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source_class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get cuadc_vision::BucketAimInfo :md5sum-) "781270f5d3447ff4d4ce3194c04670ee")
(setf (get cuadc_vision::BucketAimInfo :datatype-) "cuadc_vision/BucketAimInfo")
(setf (get cuadc_vision::BucketAimInfo :definition-)
      "# Stable bucket center and physical A/B dropper aim points.
Header header
int32 count

# True when detection, depth, intrinsics, pixel aim points and metric deltas are valid.
# NED validity is reported independently because local_pose may be unavailable.
bool valid

int32 bucket_center_x
int32 bucket_center_y

int32 a_aim_x
int32 a_aim_y
int32 b_aim_x
int32 b_aim_y

# Delta convention: delta = aim_point - image_center.
# Pixel deltas use image axes (x right, y down).
float32 a_delta_x_px
float32 a_delta_y_px
float32 b_delta_x_px
float32 b_delta_y_px

# Metric deltas are estimated from the current target depth and camera intrinsics.
float32 a_delta_x_m
float32 a_delta_y_m
float32 b_delta_x_m
float32 b_delta_y_m

# Absolute target positions in the MAVROS local NED frame.
bool a_ned_valid
float32 a_ned_n
float32 a_ned_e
float32 a_ned_d

bool b_ned_valid
float32 b_ned_n
float32 b_ned_e
float32 b_ned_d

float32 confidence
string source_class

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



(provide :cuadc_vision/BucketAimInfo "781270f5d3447ff4d4ce3194c04670ee")


