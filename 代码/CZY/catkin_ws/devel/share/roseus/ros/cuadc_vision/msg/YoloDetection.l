;; Auto-generated. Do not edit!


(when (boundp 'cuadc_vision::YoloDetection)
  (if (not (find-package "CUADC_VISION"))
    (make-package "CUADC_VISION"))
  (shadow 'YoloDetection (find-package "CUADC_VISION")))
(unless (find-package "CUADC_VISION::YOLODETECTION")
  (make-package "CUADC_VISION::YOLODETECTION"))

(in-package "ROS")
;;//! \htmlinclude YoloDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cuadc_vision::YoloDetection
  :super ros::object
  :slots (_header _detected _class_id _class_name _confidence _x_min _y_min _x_max _y_max _center_x _center_y _depth_m _position_valid _camera_x_m _camera_y_m _camera_z_m _distance_m _bbox_width_m _bbox_height_m ))

(defmethod cuadc_vision::YoloDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detected __detected) nil)
    ((:class_id __class_id) 0)
    ((:class_name __class_name) "")
    ((:confidence __confidence) 0.0)
    ((:x_min __x_min) 0)
    ((:y_min __y_min) 0)
    ((:x_max __x_max) 0)
    ((:y_max __y_max) 0)
    ((:center_x __center_x) 0)
    ((:center_y __center_y) 0)
    ((:depth_m __depth_m) 0.0)
    ((:position_valid __position_valid) nil)
    ((:camera_x_m __camera_x_m) 0.0)
    ((:camera_y_m __camera_y_m) 0.0)
    ((:camera_z_m __camera_z_m) 0.0)
    ((:distance_m __distance_m) 0.0)
    ((:bbox_width_m __bbox_width_m) 0.0)
    ((:bbox_height_m __bbox_height_m) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _detected __detected)
   (setq _class_id (round __class_id))
   (setq _class_name (string __class_name))
   (setq _confidence (float __confidence))
   (setq _x_min (round __x_min))
   (setq _y_min (round __y_min))
   (setq _x_max (round __x_max))
   (setq _y_max (round __y_max))
   (setq _center_x (round __center_x))
   (setq _center_y (round __center_y))
   (setq _depth_m (float __depth_m))
   (setq _position_valid __position_valid)
   (setq _camera_x_m (float __camera_x_m))
   (setq _camera_y_m (float __camera_y_m))
   (setq _camera_z_m (float __camera_z_m))
   (setq _distance_m (float __distance_m))
   (setq _bbox_width_m (float __bbox_width_m))
   (setq _bbox_height_m (float __bbox_height_m))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detected
   (&optional (__detected :null))
   (if (not (eq __detected :null)) (setq _detected __detected)) _detected)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:depth_m
   (&optional __depth_m)
   (if __depth_m (setq _depth_m __depth_m)) _depth_m)
  (:position_valid
   (&optional (__position_valid :null))
   (if (not (eq __position_valid :null)) (setq _position_valid __position_valid)) _position_valid)
  (:camera_x_m
   (&optional __camera_x_m)
   (if __camera_x_m (setq _camera_x_m __camera_x_m)) _camera_x_m)
  (:camera_y_m
   (&optional __camera_y_m)
   (if __camera_y_m (setq _camera_y_m __camera_y_m)) _camera_y_m)
  (:camera_z_m
   (&optional __camera_z_m)
   (if __camera_z_m (setq _camera_z_m __camera_z_m)) _camera_z_m)
  (:distance_m
   (&optional __distance_m)
   (if __distance_m (setq _distance_m __distance_m)) _distance_m)
  (:bbox_width_m
   (&optional __bbox_width_m)
   (if __bbox_width_m (setq _bbox_width_m __bbox_width_m)) _bbox_width_m)
  (:bbox_height_m
   (&optional __bbox_height_m)
   (if __bbox_height_m (setq _bbox_height_m __bbox_height_m)) _bbox_height_m)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _detected
    1
    ;; int32 _class_id
    4
    ;; string _class_name
    4 (length _class_name)
    ;; float32 _confidence
    4
    ;; int32 _x_min
    4
    ;; int32 _y_min
    4
    ;; int32 _x_max
    4
    ;; int32 _y_max
    4
    ;; int32 _center_x
    4
    ;; int32 _center_y
    4
    ;; float32 _depth_m
    4
    ;; bool _position_valid
    1
    ;; float32 _camera_x_m
    4
    ;; float32 _camera_y_m
    4
    ;; float32 _camera_z_m
    4
    ;; float32 _distance_m
    4
    ;; float32 _bbox_width_m
    4
    ;; float32 _bbox_height_m
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _detected
       (if _detected (write-byte -1 s) (write-byte 0 s))
     ;; int32 _class_id
       (write-long _class_id s)
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _x_min
       (write-long _x_min s)
     ;; int32 _y_min
       (write-long _y_min s)
     ;; int32 _x_max
       (write-long _x_max s)
     ;; int32 _y_max
       (write-long _y_max s)
     ;; int32 _center_x
       (write-long _center_x s)
     ;; int32 _center_y
       (write-long _center_y s)
     ;; float32 _depth_m
       (sys::poke _depth_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _position_valid
       (if _position_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _camera_x_m
       (sys::poke _camera_x_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _camera_y_m
       (sys::poke _camera_y_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _camera_z_m
       (sys::poke _camera_z_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance_m
       (sys::poke _distance_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_width_m
       (sys::poke _bbox_width_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_height_m
       (sys::poke _bbox_height_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _detected
     (setq _detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _class_id
     (setq _class_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _x_min
     (setq _x_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_min
     (setq _y_min (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x_max
     (setq _x_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_max
     (setq _y_max (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_x
     (setq _center_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_y
     (setq _center_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _depth_m
     (setq _depth_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _position_valid
     (setq _position_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _camera_x_m
     (setq _camera_x_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _camera_y_m
     (setq _camera_y_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _camera_z_m
     (setq _camera_z_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance_m
     (setq _distance_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_width_m
     (setq _bbox_width_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_height_m
     (setq _bbox_height_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cuadc_vision::YoloDetection :md5sum-) "c97190c54505fa4fe0a0e57bb2112989")
(setf (get cuadc_vision::YoloDetection :datatype-) "cuadc_vision/YoloDetection")
(setf (get cuadc_vision::YoloDetection :definition-)
      "Header header
bool detected
int32 class_id
string class_name
float32 confidence
int32 x_min
int32 y_min
int32 x_max
int32 y_max
int32 center_x
int32 center_y
float32 depth_m
bool position_valid
float32 camera_x_m
float32 camera_y_m
float32 camera_z_m
float32 distance_m
float32 bbox_width_m
float32 bbox_height_m

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



(provide :cuadc_vision/YoloDetection "c97190c54505fa4fe0a0e57bb2112989")


