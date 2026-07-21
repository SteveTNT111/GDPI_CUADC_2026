
(cl:in-package :asdf)

(defsystem "d435i_yellow_circle_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GeoTarget" :depends-on ("_package_GeoTarget"))
    (:file "_package_GeoTarget" :depends-on ("_package"))
    (:file "MissionTarget" :depends-on ("_package_MissionTarget"))
    (:file "_package_MissionTarget" :depends-on ("_package"))
    (:file "YellowCircle" :depends-on ("_package_YellowCircle"))
    (:file "_package_YellowCircle" :depends-on ("_package"))
    (:file "YoloDetection" :depends-on ("_package_YoloDetection"))
    (:file "_package_YoloDetection" :depends-on ("_package"))
    (:file "YoloDetections" :depends-on ("_package_YoloDetections"))
    (:file "_package_YoloDetections" :depends-on ("_package"))
  ))