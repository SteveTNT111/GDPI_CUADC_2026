;; Auto-generated. Do not edit!


(when (boundp 'd435i_yellow_circle_detector::YoloDetections)
  (if (not (find-package "D435I_YELLOW_CIRCLE_DETECTOR"))
    (make-package "D435I_YELLOW_CIRCLE_DETECTOR"))
  (shadow 'YoloDetections (find-package "D435I_YELLOW_CIRCLE_DETECTOR")))
(unless (find-package "D435I_YELLOW_CIRCLE_DETECTOR::YOLODETECTIONS")
  (make-package "D435I_YELLOW_CIRCLE_DETECTOR::YOLODETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude YoloDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass d435i_yellow_circle_detector::YoloDetections
  :super ros::object
  :slots (_header _detections ))

(defmethod d435i_yellow_circle_detector::YoloDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; d435i_yellow_circle_detector/YoloDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; d435i_yellow_circle_detector/YoloDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; d435i_yellow_circle_detector/YoloDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance d435i_yellow_circle_detector::YoloDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get d435i_yellow_circle_detector::YoloDetections :md5sum-) "a930238f1ca26e45c4d2a892c5bf7982")
(setf (get d435i_yellow_circle_detector::YoloDetections :datatype-) "d435i_yellow_circle_detector/YoloDetections")
(setf (get d435i_yellow_circle_detector::YoloDetections :definition-)
      "Header header
YoloDetection[] detections

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

================================================================================
MSG: d435i_yellow_circle_detector/YoloDetection
Header header
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

")



(provide :d435i_yellow_circle_detector/YoloDetections "a930238f1ca26e45c4d2a892c5bf7982")


