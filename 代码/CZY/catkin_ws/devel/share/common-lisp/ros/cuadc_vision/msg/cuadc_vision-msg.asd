
(cl:in-package :asdf)

(defsystem "cuadc_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BucketInfo" :depends-on ("_package_BucketInfo"))
    (:file "_package_BucketInfo" :depends-on ("_package"))
    (:file "GeoTarget" :depends-on ("_package_GeoTarget"))
    (:file "_package_GeoTarget" :depends-on ("_package"))
    (:file "MissionStatus" :depends-on ("_package_MissionStatus"))
    (:file "_package_MissionStatus" :depends-on ("_package"))
    (:file "YoloDetection" :depends-on ("_package_YoloDetection"))
    (:file "_package_YoloDetection" :depends-on ("_package"))
    (:file "YoloDetections" :depends-on ("_package_YoloDetections"))
    (:file "_package_YoloDetections" :depends-on ("_package"))
  ))