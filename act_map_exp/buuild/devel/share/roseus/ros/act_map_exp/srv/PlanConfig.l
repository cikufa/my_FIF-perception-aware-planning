;; Auto-generated. Do not edit!


(when (boundp 'act_map_exp::PlanConfig)
  (if (not (find-package "ACT_MAP_EXP"))
    (make-package "ACT_MAP_EXP"))
  (shadow 'PlanConfig (find-package "ACT_MAP_EXP")))
(unless (find-package "ACT_MAP_EXP::PLANCONFIG")
  (make-package "ACT_MAP_EXP::PLANCONFIG"))
(unless (find-package "ACT_MAP_EXP::PLANCONFIGREQUEST")
  (make-package "ACT_MAP_EXP::PLANCONFIGREQUEST"))
(unless (find-package "ACT_MAP_EXP::PLANCONFIGRESPONSE")
  (make-package "ACT_MAP_EXP::PLANCONFIGRESPONSE"))

(in-package "ROS")





(defclass act_map_exp::PlanConfigRequest
  :super ros::object
  :slots (_config ))

(defmethod act_map_exp::PlanConfigRequest
  (:init
   (&key
    ((:config __config) "")
    )
   (send-super :init)
   (setq _config (string __config))
   self)
  (:config
   (&optional __config)
   (if __config (setq _config __config)) _config)
  (:serialization-length
   ()
   (+
    ;; string _config
    4 (length _config)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _config
       (write-long (length _config) s) (princ _config s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _config
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _config (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass act_map_exp::PlanConfigResponse
  :super ros::object
  :slots ())

(defmethod act_map_exp::PlanConfigResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass act_map_exp::PlanConfig
  :super ros::object
  :slots ())

(setf (get act_map_exp::PlanConfig :md5sum-) "b3532af339db184b4a6a974d00ee4fe6")
(setf (get act_map_exp::PlanConfig :datatype-) "act_map_exp/PlanConfig")
(setf (get act_map_exp::PlanConfig :request) act_map_exp::PlanConfigRequest)
(setf (get act_map_exp::PlanConfig :response) act_map_exp::PlanConfigResponse)

(defmethod act_map_exp::PlanConfigRequest
  (:response () (instance act_map_exp::PlanConfigResponse :init)))

(setf (get act_map_exp::PlanConfigRequest :md5sum-) "b3532af339db184b4a6a974d00ee4fe6")
(setf (get act_map_exp::PlanConfigRequest :datatype-) "act_map_exp/PlanConfigRequest")
(setf (get act_map_exp::PlanConfigRequest :definition-)
      "string config
---

")

(setf (get act_map_exp::PlanConfigResponse :md5sum-) "b3532af339db184b4a6a974d00ee4fe6")
(setf (get act_map_exp::PlanConfigResponse :datatype-) "act_map_exp/PlanConfigResponse")
(setf (get act_map_exp::PlanConfigResponse :definition-)
      "string config
---

")



(provide :act_map_exp/PlanConfig "b3532af339db184b4a6a974d00ee4fe6")


