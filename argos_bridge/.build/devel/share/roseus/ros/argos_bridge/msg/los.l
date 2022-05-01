;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::los)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'los (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::LOS")
  (make-package "ARGOS_BRIDGE::LOS"))

(in-package "ROS")
;;//! \htmlinclude los.msg.html


(defclass argos_bridge::los
  :super ros::object
  :slots (_robotName ))

(defmethod argos_bridge::los
  (:init
   (&key
    ((:robotName __robotName) "")
    )
   (send-super :init)
   (setq _robotName (string __robotName))
   self)
  (:robotName
   (&optional __robotName)
   (if __robotName (setq _robotName __robotName)) _robotName)
  (:serialization-length
   ()
   (+
    ;; string _robotName
    4 (length _robotName)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robotName
       (write-long (length _robotName) s) (princ _robotName s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robotName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robotName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get argos_bridge::los :md5sum-) "0e4ce7af4736710e228ed1cbe6f009e7")
(setf (get argos_bridge::los :datatype-) "argos_bridge/los")
(setf (get argos_bridge::los :definition-)
      "string robotName

")



(provide :argos_bridge/los "0e4ce7af4736710e228ed1cbe6f009e7")


