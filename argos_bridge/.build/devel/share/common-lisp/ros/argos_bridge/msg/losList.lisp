; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude losList.msg.html

(cl:defclass <losList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (n
    :reader n
    :initarg :n
    :type cl:integer
    :initform 0)
   (robots
    :reader robots
    :initarg :robots
    :type (cl:vector argos_bridge-msg:los)
   :initform (cl:make-array 0 :element-type 'argos_bridge-msg:los :initial-element (cl:make-instance 'argos_bridge-msg:los))))
)

(cl:defclass losList (<losList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <losList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'losList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<losList> is deprecated: use argos_bridge-msg:losList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <losList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:header-val is deprecated.  Use argos_bridge-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <losList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:n-val is deprecated.  Use argos_bridge-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'robots-val :lambda-list '(m))
(cl:defmethod robots-val ((m <losList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:robots-val is deprecated.  Use argos_bridge-msg:robots instead.")
  (robots m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <losList>) ostream)
  "Serializes a message object of type '<losList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'n)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robots))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'robots))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <losList>) istream)
  "Deserializes a message object of type '<losList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robots) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robots)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'argos_bridge-msg:los))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<losList>)))
  "Returns string type for a message object of type '<losList>"
  "argos_bridge/losList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'losList)))
  "Returns string type for a message object of type 'losList"
  "argos_bridge/losList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<losList>)))
  "Returns md5sum for a message object of type '<losList>"
  "3c2be5cd3cebc9db2ce990dc41ef741c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'losList)))
  "Returns md5sum for a message object of type 'losList"
  "3c2be5cd3cebc9db2ce990dc41ef741c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<losList>)))
  "Returns full string definition for message of type '<losList>"
  (cl:format cl:nil "Header header~%int32 n~%los[] robots~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: argos_bridge/los~%string robotName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'losList)))
  "Returns full string definition for message of type 'losList"
  (cl:format cl:nil "Header header~%int32 n~%los[] robots~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: argos_bridge/los~%string robotName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <losList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robots) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <losList>))
  "Converts a ROS message object to a list"
  (cl:list 'losList
    (cl:cons ':header (header msg))
    (cl:cons ':n (n msg))
    (cl:cons ':robots (robots msg))
))
