;; Auto-generated. Do not edit!


(when (boundp 'cuadc_vision::MissionStatus)
  (if (not (find-package "CUADC_VISION"))
    (make-package "CUADC_VISION"))
  (shadow 'MissionStatus (find-package "CUADC_VISION")))
(unless (find-package "CUADC_VISION::MISSIONSTATUS")
  (make-package "CUADC_VISION::MISSIONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude MissionStatus.msg.html


(defclass cuadc_vision::MissionStatus
  :super ros::object
  :slots (_ammo_a _ammo_b _aiming _last_drop ))

(defmethod cuadc_vision::MissionStatus
  (:init
   (&key
    ((:ammo_a __ammo_a) 0)
    ((:ammo_b __ammo_b) 0)
    ((:aiming __aiming) nil)
    ((:last_drop __last_drop) "")
    )
   (send-super :init)
   (setq _ammo_a (round __ammo_a))
   (setq _ammo_b (round __ammo_b))
   (setq _aiming __aiming)
   (setq _last_drop (string __last_drop))
   self)
  (:ammo_a
   (&optional __ammo_a)
   (if __ammo_a (setq _ammo_a __ammo_a)) _ammo_a)
  (:ammo_b
   (&optional __ammo_b)
   (if __ammo_b (setq _ammo_b __ammo_b)) _ammo_b)
  (:aiming
   (&optional (__aiming :null))
   (if (not (eq __aiming :null)) (setq _aiming __aiming)) _aiming)
  (:last_drop
   (&optional __last_drop)
   (if __last_drop (setq _last_drop __last_drop)) _last_drop)
  (:serialization-length
   ()
   (+
    ;; uint8 _ammo_a
    1
    ;; uint8 _ammo_b
    1
    ;; bool _aiming
    1
    ;; string _last_drop
    4 (length _last_drop)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _ammo_a
       (write-byte _ammo_a s)
     ;; uint8 _ammo_b
       (write-byte _ammo_b s)
     ;; bool _aiming
       (if _aiming (write-byte -1 s) (write-byte 0 s))
     ;; string _last_drop
       (write-long (length _last_drop) s) (princ _last_drop s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _ammo_a
     (setq _ammo_a (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _ammo_b
     (setq _ammo_b (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _aiming
     (setq _aiming (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _last_drop
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _last_drop (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get cuadc_vision::MissionStatus :md5sum-) "42a0745a697dd6adddaac7f2c09f5f59")
(setf (get cuadc_vision::MissionStatus :datatype-) "cuadc_vision/MissionStatus")
(setf (get cuadc_vision::MissionStatus :definition-)
      "# MissionStatus.msg — main.py → detector_node 任务状态同步
#
# 用于 detector_node 在画面上显示弹药、瞄准、抛投等任务状态信息。

uint8 ammo_a                # 前抛投器 (A) 剩余弹药数，0 表示无/未挂载
uint8 ammo_b                # 后抛投器 (B) 剩余弹药数，0 表示无/未挂载
bool aiming                 # 飞控处于 GUIDED 模式且正在执行对准任务
string last_drop            # 最近一次抛投的抛投器编号 (\"A\" / \"B\" / \"\")

")



(provide :cuadc_vision/MissionStatus "42a0745a697dd6adddaac7f2c09f5f59")


