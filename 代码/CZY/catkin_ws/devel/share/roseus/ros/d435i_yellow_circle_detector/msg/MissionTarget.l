;; Auto-generated. Do not edit!


(when (boundp 'd435i_yellow_circle_detector::MissionTarget)
  (if (not (find-package "D435I_YELLOW_CIRCLE_DETECTOR"))
    (make-package "D435I_YELLOW_CIRCLE_DETECTOR"))
  (shadow 'MissionTarget (find-package "D435I_YELLOW_CIRCLE_DETECTOR")))
(unless (find-package "D435I_YELLOW_CIRCLE_DETECTOR::MISSIONTARGET")
  (make-package "D435I_YELLOW_CIRCLE_DETECTOR::MISSIONTARGET"))

(in-package "ROS")
;;//! \htmlinclude MissionTarget.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass d435i_yellow_circle_detector::MissionTarget
  :super ros::object
  :slots (_header _valid _mission_stage _target_type _class_name _confidence _center_x _center_y _camera_x_m _camera_y_m _camera_z_m _distance_m _bbox_width_m _bbox_height_m _nominal_diameter_m _diameter_class _zone_hint _a_zone_radius_m _b_zone_radius_m _action_hint ))

(defmethod d435i_yellow_circle_detector::MissionTarget
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:valid __valid) nil)
    ((:mission_stage __mission_stage) "")
    ((:target_type __target_type) "")
    ((:class_name __class_name) "")
    ((:confidence __confidence) 0.0)
    ((:center_x __center_x) 0)
    ((:center_y __center_y) 0)
    ((:camera_x_m __camera_x_m) 0.0)
    ((:camera_y_m __camera_y_m) 0.0)
    ((:camera_z_m __camera_z_m) 0.0)
    ((:distance_m __distance_m) 0.0)
    ((:bbox_width_m __bbox_width_m) 0.0)
    ((:bbox_height_m __bbox_height_m) 0.0)
    ((:nominal_diameter_m __nominal_diameter_m) 0.0)
    ((:diameter_class __diameter_class) "")
    ((:zone_hint __zone_hint) "")
    ((:a_zone_radius_m __a_zone_radius_m) 0.0)
    ((:b_zone_radius_m __b_zone_radius_m) 0.0)
    ((:action_hint __action_hint) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _valid __valid)
   (setq _mission_stage (string __mission_stage))
   (setq _target_type (string __target_type))
   (setq _class_name (string __class_name))
   (setq _confidence (float __confidence))
   (setq _center_x (round __center_x))
   (setq _center_y (round __center_y))
   (setq _camera_x_m (float __camera_x_m))
   (setq _camera_y_m (float __camera_y_m))
   (setq _camera_z_m (float __camera_z_m))
   (setq _distance_m (float __distance_m))
   (setq _bbox_width_m (float __bbox_width_m))
   (setq _bbox_height_m (float __bbox_height_m))
   (setq _nominal_diameter_m (float __nominal_diameter_m))
   (setq _diameter_class (string __diameter_class))
   (setq _zone_hint (string __zone_hint))
   (setq _a_zone_radius_m (float __a_zone_radius_m))
   (setq _b_zone_radius_m (float __b_zone_radius_m))
   (setq _action_hint (string __action_hint))
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
  (:mission_stage
   (&optional __mission_stage)
   (if __mission_stage (setq _mission_stage __mission_stage)) _mission_stage)
  (:target_type
   (&optional __target_type)
   (if __target_type (setq _target_type __target_type)) _target_type)
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
  (:distance_m
   (&optional __distance_m)
   (if __distance_m (setq _distance_m __distance_m)) _distance_m)
  (:bbox_width_m
   (&optional __bbox_width_m)
   (if __bbox_width_m (setq _bbox_width_m __bbox_width_m)) _bbox_width_m)
  (:bbox_height_m
   (&optional __bbox_height_m)
   (if __bbox_height_m (setq _bbox_height_m __bbox_height_m)) _bbox_height_m)
  (:nominal_diameter_m
   (&optional __nominal_diameter_m)
   (if __nominal_diameter_m (setq _nominal_diameter_m __nominal_diameter_m)) _nominal_diameter_m)
  (:diameter_class
   (&optional __diameter_class)
   (if __diameter_class (setq _diameter_class __diameter_class)) _diameter_class)
  (:zone_hint
   (&optional __zone_hint)
   (if __zone_hint (setq _zone_hint __zone_hint)) _zone_hint)
  (:a_zone_radius_m
   (&optional __a_zone_radius_m)
   (if __a_zone_radius_m (setq _a_zone_radius_m __a_zone_radius_m)) _a_zone_radius_m)
  (:b_zone_radius_m
   (&optional __b_zone_radius_m)
   (if __b_zone_radius_m (setq _b_zone_radius_m __b_zone_radius_m)) _b_zone_radius_m)
  (:action_hint
   (&optional __action_hint)
   (if __action_hint (setq _action_hint __action_hint)) _action_hint)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _valid
    1
    ;; string _mission_stage
    4 (length _mission_stage)
    ;; string _target_type
    4 (length _target_type)
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
    ;; float32 _distance_m
    4
    ;; float32 _bbox_width_m
    4
    ;; float32 _bbox_height_m
    4
    ;; float32 _nominal_diameter_m
    4
    ;; string _diameter_class
    4 (length _diameter_class)
    ;; string _zone_hint
    4 (length _zone_hint)
    ;; float32 _a_zone_radius_m
    4
    ;; float32 _b_zone_radius_m
    4
    ;; string _action_hint
    4 (length _action_hint)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; string _mission_stage
       (write-long (length _mission_stage) s) (princ _mission_stage s)
     ;; string _target_type
       (write-long (length _target_type) s) (princ _target_type s)
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
     ;; float32 _distance_m
       (sys::poke _distance_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_width_m
       (sys::poke _bbox_width_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbox_height_m
       (sys::poke _bbox_height_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _nominal_diameter_m
       (sys::poke _nominal_diameter_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _diameter_class
       (write-long (length _diameter_class) s) (princ _diameter_class s)
     ;; string _zone_hint
       (write-long (length _zone_hint) s) (princ _zone_hint s)
     ;; float32 _a_zone_radius_m
       (sys::poke _a_zone_radius_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b_zone_radius_m
       (sys::poke _b_zone_radius_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _action_hint
       (write-long (length _action_hint) s) (princ _action_hint s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _mission_stage
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mission_stage (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _target_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
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
   ;; float32 _distance_m
     (setq _distance_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_width_m
     (setq _bbox_width_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbox_height_m
     (setq _bbox_height_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _nominal_diameter_m
     (setq _nominal_diameter_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _diameter_class
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _diameter_class (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _zone_hint
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _zone_hint (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _a_zone_radius_m
     (setq _a_zone_radius_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b_zone_radius_m
     (setq _b_zone_radius_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _action_hint
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action_hint (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get d435i_yellow_circle_detector::MissionTarget :md5sum-) "9e275971ddc71593e97b9bdc197f8cad")
(setf (get d435i_yellow_circle_detector::MissionTarget :datatype-) "d435i_yellow_circle_detector/MissionTarget")
(setf (get d435i_yellow_circle_detector::MissionTarget :definition-)
      "Header header
bool valid
string mission_stage
string target_type
string class_name
float32 confidence
int32 center_x
int32 center_y
float32 camera_x_m
float32 camera_y_m
float32 camera_z_m
float32 distance_m
float32 bbox_width_m
float32 bbox_height_m
float32 nominal_diameter_m
string diameter_class
string zone_hint
float32 a_zone_radius_m
float32 b_zone_radius_m
string action_hint

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



(provide :d435i_yellow_circle_detector/MissionTarget "9e275971ddc71593e97b9bdc197f8cad")


