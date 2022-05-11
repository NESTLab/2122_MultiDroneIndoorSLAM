;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::losList)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'losList (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::LOSLIST")
  (make-package "ARGOS_BRIDGE::LOSLIST"))

(in-package "ROS")
;;//! \htmlinclude losList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass argos_bridge::losList
  :super ros::object
  :slots (_header _n _robots ))

(defmethod argos_bridge::losList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:n __n) 0)
    ((:robots __robots) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _n (round __n))
   (setq _robots __robots)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:robots
   (&rest __robots)
   (if (keywordp (car __robots))
       (send* _robots __robots)
     (progn
       (if __robots (setq _robots (car __robots)))
       _robots)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _n
    4
    ;; argos_bridge/los[] _robots
    (apply #'+ (send-all _robots :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _n
       (write-long _n s)
     ;; argos_bridge/los[] _robots
     (write-long (length _robots) s)
     (dolist (elem _robots)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _n
     (setq _n (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; argos_bridge/los[] _robots
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robots (let (r) (dotimes (i n) (push (instance argos_bridge::los :init) r)) r))
     (dolist (elem- _robots)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get argos_bridge::losList :md5sum-) "3c2be5cd3cebc9db2ce990dc41ef741c")
(setf (get argos_bridge::losList :datatype-) "argos_bridge/losList")
(setf (get argos_bridge::losList :definition-)
      "Header header
int32 n
los[] robots

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
MSG: argos_bridge/los
string robotName

")



(provide :argos_bridge/losList "3c2be5cd3cebc9db2ce990dc41ef741c")


