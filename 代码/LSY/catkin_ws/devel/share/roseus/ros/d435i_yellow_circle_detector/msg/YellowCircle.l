;; Auto-generated. Do not edit!


(when (boundp 'd435i_yellow_circle_detector::YellowCircle)
  (if (not (find-package "D435I_YELLOW_CIRCLE_DETECTOR"))
    (make-package "D435I_YELLOW_CIRCLE_DETECTOR"))
  (shadow 'YellowCircle (find-package "D435I_YELLOW_CIRCLE_DETECTOR")))
(unless (find-package "D435I_YELLOW_CIRCLE_DETECTOR::YELLOWCIRCLE")
  (make-package "D435I_YELLOW_CIRCLE_DETECTOR::YELLOWCIRCLE"))

(in-package "ROS")
;;//! \htmlinclude YellowCircle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass d435i_yellow_circle_detector::YellowCircle
  :super ros::object
  :slots (_header _detected _x _y _radius _area _depth_m _raw_depth_m _diameter_depth_m _calibrated_depth_m _depth_source _center_offset_x _center_offset_y _position_valid _camera_x_m _camera_y_m _camera_z_m _distance_m _body_x_m _body_y_m _body_z_m _body_distance_m ))

(defmethod d435i_yellow_circle_detector::YellowCircle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detected __detected) nil)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:radius __radius) 0.0)
    ((:area __area) 0.0)
    ((:depth_m __depth_m) 0.0)
    ((:raw_depth_m __raw_depth_m) 0.0)
    ((:diameter_depth_m __diameter_depth_m) 0.0)
    ((:calibrated_depth_m __calibrated_depth_m) 0.0)
    ((:depth_source __depth_source) "")
    ((:center_offset_x __center_offset_x) 0.0)
    ((:center_offset_y __center_offset_y) 0.0)
    ((:position_valid __position_valid) nil)
    ((:camera_x_m __camera_x_m) 0.0)
    ((:camera_y_m __camera_y_m) 0.0)
    ((:camera_z_m __camera_z_m) 0.0)
    ((:distance_m __distance_m) 0.0)
    ((:body_x_m __body_x_m) 0.0)
    ((:body_y_m __body_y_m) 0.0)
    ((:body_z_m __body_z_m) 0.0)
    ((:body_distance_m __body_distance_m) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _detected __detected)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _radius (float __radius))
   (setq _area (float __area))
   (setq _depth_m (float __depth_m))
   (setq _raw_depth_m (float __raw_depth_m))
   (setq _diameter_depth_m (float __diameter_depth_m))
   (setq _calibrated_depth_m (float __calibrated_depth_m))
   (setq _depth_source (string __depth_source))
   (setq _center_offset_x (float __center_offset_x))
   (setq _center_offset_y (float __center_offset_y))
   (setq _position_valid __position_valid)
   (setq _camera_x_m (float __camera_x_m))
   (setq _camera_y_m (float __camera_y_m))
   (setq _camera_z_m (float __camera_z_m))
   (setq _distance_m (float __distance_m))
   (setq _body_x_m (float __body_x_m))
   (setq _body_y_m (float __body_y_m))
   (setq _body_z_m (float __body_z_m))
   (setq _body_distance_m (float __body_distance_m))
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
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:depth_m
   (&optional __depth_m)
   (if __depth_m (setq _depth_m __depth_m)) _depth_m)
  (:raw_depth_m
   (&optional __raw_depth_m)
   (if __raw_depth_m (setq _raw_depth_m __raw_depth_m)) _raw_depth_m)
  (:diameter_depth_m
   (&optional __diameter_depth_m)
   (if __diameter_depth_m (setq _diameter_depth_m __diameter_depth_m)) _diameter_depth_m)
  (:calibrated_depth_m
   (&optional __calibrated_depth_m)
   (if __calibrated_depth_m (setq _calibrated_depth_m __calibrated_depth_m)) _calibrated_depth_m)
  (:depth_source
   (&optional __depth_source)
   (if __depth_source (setq _depth_source __depth_source)) _depth_source)
  (:center_offset_x
   (&optional __center_offset_x)
   (if __center_offset_x (setq _center_offset_x __center_offset_x)) _center_offset_x)
  (:center_offset_y
   (&optional __center_offset_y)
   (if __center_offset_y (setq _center_offset_y __center_offset_y)) _center_offset_y)
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
  (:body_x_m
   (&optional __body_x_m)
   (if __body_x_m (setq _body_x_m __body_x_m)) _body_x_m)
  (:body_y_m
   (&optional __body_y_m)
   (if __body_y_m (setq _body_y_m __body_y_m)) _body_y_m)
  (:body_z_m
   (&optional __body_z_m)
   (if __body_z_m (setq _body_z_m __body_z_m)) _body_z_m)
  (:body_distance_m
   (&optional __body_distance_m)
   (if __body_distance_m (setq _body_distance_m __body_distance_m)) _body_distance_m)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _detected
    1
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; float32 _radius
    4
    ;; float32 _area
    4
    ;; float32 _depth_m
    4
    ;; float32 _raw_depth_m
    4
    ;; float32 _diameter_depth_m
    4
    ;; float32 _calibrated_depth_m
    4
    ;; string _depth_source
    4 (length _depth_source)
    ;; float32 _center_offset_x
    4
    ;; float32 _center_offset_y
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
    ;; float32 _body_x_m
    4
    ;; float32 _body_y_m
    4
    ;; float32 _body_z_m
    4
    ;; float32 _body_distance_m
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
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _area
       (sys::poke _area (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _depth_m
       (sys::poke _depth_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _raw_depth_m
       (sys::poke _raw_depth_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _diameter_depth_m
       (sys::poke _diameter_depth_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _calibrated_depth_m
       (sys::poke _calibrated_depth_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _depth_source
       (write-long (length _depth_source) s) (princ _depth_source s)
     ;; float32 _center_offset_x
       (sys::poke _center_offset_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _center_offset_y
       (sys::poke _center_offset_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
     ;; float32 _body_x_m
       (sys::poke _body_x_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _body_y_m
       (sys::poke _body_y_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _body_z_m
       (sys::poke _body_z_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _body_distance_m
       (sys::poke _body_distance_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _detected
     (setq _detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _area
     (setq _area (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _depth_m
     (setq _depth_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _raw_depth_m
     (setq _raw_depth_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _diameter_depth_m
     (setq _diameter_depth_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _calibrated_depth_m
     (setq _calibrated_depth_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _depth_source
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _depth_source (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _center_offset_x
     (setq _center_offset_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _center_offset_y
     (setq _center_offset_y (sys::peek buf ptr- :float)) (incf ptr- 4)
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
   ;; float32 _body_x_m
     (setq _body_x_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _body_y_m
     (setq _body_y_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _body_z_m
     (setq _body_z_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _body_distance_m
     (setq _body_distance_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get d435i_yellow_circle_detector::YellowCircle :md5sum-) "546a21369cee59d7afb5df9576c1da21")
(setf (get d435i_yellow_circle_detector::YellowCircle :datatype-) "d435i_yellow_circle_detector/YellowCircle")
(setf (get d435i_yellow_circle_detector::YellowCircle :definition-)
      "Header header
bool detected
int32 x
int32 y
float32 radius
float32 area
float32 depth_m
float32 raw_depth_m
float32 diameter_depth_m
float32 calibrated_depth_m
string depth_source
float32 center_offset_x
float32 center_offset_y
bool position_valid
float32 camera_x_m
float32 camera_y_m
float32 camera_z_m
float32 distance_m
float32 body_x_m
float32 body_y_m
float32 body_z_m
float32 body_distance_m

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



(provide :d435i_yellow_circle_detector/YellowCircle "546a21369cee59d7afb5df9576c1da21")


