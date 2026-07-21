;; Auto-generated. Do not edit!


(when (boundp 'd435i_yellow_circle_detector::GeoTarget)
  (if (not (find-package "D435I_YELLOW_CIRCLE_DETECTOR"))
    (make-package "D435I_YELLOW_CIRCLE_DETECTOR"))
  (shadow 'GeoTarget (find-package "D435I_YELLOW_CIRCLE_DETECTOR")))
(unless (find-package "D435I_YELLOW_CIRCLE_DETECTOR::GEOTARGET")
  (make-package "D435I_YELLOW_CIRCLE_DETECTOR::GEOTARGET"))

(in-package "ROS")
;;//! \htmlinclude GeoTarget.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass d435i_yellow_circle_detector::GeoTarget
  :super ros::object
  :slots (_header _valid _status _source_topic _class_name _confidence _center_x _center_y _camera_x_m _camera_y_m _camera_z_m _body_x_m _body_y_m _body_z_m _enu_east_m _enu_north_m _enu_up_m _latitude _longitude _altitude ))

(defmethod d435i_yellow_circle_detector::GeoTarget
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:valid __valid) nil)
    ((:status __status) "")
    ((:source_topic __source_topic) "")
    ((:class_name __class_name) "")
    ((:confidence __confidence) 0.0)
    ((:center_x __center_x) 0)
    ((:center_y __center_y) 0)
    ((:camera_x_m __camera_x_m) 0.0)
    ((:camera_y_m __camera_y_m) 0.0)
    ((:camera_z_m __camera_z_m) 0.0)
    ((:body_x_m __body_x_m) 0.0)
    ((:body_y_m __body_y_m) 0.0)
    ((:body_z_m __body_z_m) 0.0)
    ((:enu_east_m __enu_east_m) 0.0)
    ((:enu_north_m __enu_north_m) 0.0)
    ((:enu_up_m __enu_up_m) 0.0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _valid __valid)
   (setq _status (string __status))
   (setq _source_topic (string __source_topic))
   (setq _class_name (string __class_name))
   (setq _confidence (float __confidence))
   (setq _center_x (round __center_x))
   (setq _center_y (round __center_y))
   (setq _camera_x_m (float __camera_x_m))
   (setq _camera_y_m (float __camera_y_m))
   (setq _camera_z_m (float __camera_z_m))
   (setq _body_x_m (float __body_x_m))
   (setq _body_y_m (float __body_y_m))
   (setq _body_z_m (float __body_z_m))
   (setq _enu_east_m (float __enu_east_m))
   (setq _enu_north_m (float __enu_north_m))
   (setq _enu_up_m (float __enu_up_m))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:source_topic
   (&optional __source_topic)
   (if __source_topic (setq _source_topic __source_topic)) _source_topic)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:camera_x_m
   (&optional __camera_x_m)
   (if __camera_x_m (setq _camera_x_m __camera_x_m)) _camera_x_m)
  (:camera_y_m
   (&optional __camera_y_m)
   (if __camera_y_m (setq _camera_y_m __camera_y_m)) _camera_y_m)
  (:camera_z_m
   (&optional __camera_z_m)
   (if __camera_z_m (setq _camera_z_m __camera_z_m)) _camera_z_m)
  (:body_x_m
   (&optional __body_x_m)
   (if __body_x_m (setq _body_x_m __body_x_m)) _body_x_m)
  (:body_y_m
   (&optional __body_y_m)
   (if __body_y_m (setq _body_y_m __body_y_m)) _body_y_m)
  (:body_z_m
   (&optional __body_z_m)
   (if __body_z_m (setq _body_z_m __body_z_m)) _body_z_m)
  (:enu_east_m
   (&optional __enu_east_m)
   (if __enu_east_m (setq _enu_east_m __enu_east_m)) _enu_east_m)
  (:enu_north_m
   (&optional __enu_north_m)
   (if __enu_north_m (setq _enu_north_m __enu_north_m)) _enu_north_m)
  (:enu_up_m
   (&optional __enu_up_m)
   (if __enu_up_m (setq _enu_up_m __enu_up_m)) _enu_up_m)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _valid
    1
    ;; string _status
    4 (length _status)
    ;; string _source_topic
    4 (length _source_topic)
    ;; string _class_name
    4 (length _class_name)
    ;; float32 _confidence
    4
    ;; int32 _center_x
    4
    ;; int32 _center_y
    4
    ;; float32 _camera_x_m
    4
    ;; float32 _camera_y_m
    4
    ;; float32 _camera_z_m
    4
    ;; float32 _body_x_m
    4
    ;; float32 _body_y_m
    4
    ;; float32 _body_z_m
    4
    ;; float32 _enu_east_m
    4
    ;; float32 _enu_north_m
    4
    ;; float32 _enu_up_m
    4
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; string _source_topic
       (write-long (length _source_topic) s) (princ _source_topic s)
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _center_x
       (write-long _center_x s)
     ;; int32 _center_y
       (write-long _center_y s)
     ;; float32 _camera_x_m
       (sys::poke _camera_x_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _camera_y_m
       (sys::poke _camera_y_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _camera_z_m
       (sys::poke _camera_z_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _body_x_m
       (sys::poke _body_x_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _body_y_m
       (sys::poke _body_y_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _body_z_m
       (sys::poke _body_z_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _enu_east_m
       (sys::poke _enu_east_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _enu_north_m
       (sys::poke _enu_north_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _enu_up_m
       (sys::poke _enu_up_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _source_topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source_topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _center_x
     (setq _center_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _center_y
     (setq _center_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _camera_x_m
     (setq _camera_x_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _camera_y_m
     (setq _camera_y_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _camera_z_m
     (setq _camera_z_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _body_x_m
     (setq _body_x_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _body_y_m
     (setq _body_y_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _body_z_m
     (setq _body_z_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _enu_east_m
     (setq _enu_east_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _enu_north_m
     (setq _enu_north_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _enu_up_m
     (setq _enu_up_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get d435i_yellow_circle_detector::GeoTarget :md5sum-) "97435bcc3e1e35e6ba97ca5a2f45aeba")
(setf (get d435i_yellow_circle_detector::GeoTarget :datatype-) "d435i_yellow_circle_detector/GeoTarget")
(setf (get d435i_yellow_circle_detector::GeoTarget :definition-)
      "Header header
bool valid
string status
string source_topic
string class_name
float32 confidence
int32 center_x
int32 center_y
float32 camera_x_m
float32 camera_y_m
float32 camera_z_m
float32 body_x_m
float32 body_y_m
float32 body_z_m
float32 enu_east_m
float32 enu_north_m
float32 enu_up_m
float64 latitude
float64 longitude
float64 altitude

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



(provide :d435i_yellow_circle_detector/GeoTarget "97435bcc3e1e35e6ba97ca5a2f45aeba")


