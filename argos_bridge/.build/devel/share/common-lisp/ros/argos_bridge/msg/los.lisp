; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude los.msg.html

(cl:defclass <los> (roslisp-msg-protocol:ros-message)
  ((robotName
    :reader robotName
    :initarg :robotName
    :type cl:string
    :initform ""))
)

(cl:defclass los (<los>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <los>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'los)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<los> is deprecated: use argos_bridge-msg:los instead.")))

(cl:ensure-generic-function 'robotName-val :lambda-list '(m))
(cl:defmethod robotName-val ((m <los>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:robotName-val is deprecated.  Use argos_bridge-msg:robotName instead.")
  (robotName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <los>) ostream)
  "Serializes a message object of type '<los>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robotName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robotName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <los>) istream)
  "Deserializes a message object of type '<los>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robotName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robotName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<los>)))
  "Returns string type for a message object of type '<los>"
  "argos_bridge/los")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'los)))
  "Returns string type for a message object of type 'los"
  "argos_bridge/los")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<los>)))
  "Returns md5sum for a message object of type '<los>"
  "0e4ce7af4736710e228ed1cbe6f009e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'los)))
  "Returns md5sum for a message object of type 'los"
  "0e4ce7af4736710e228ed1cbe6f009e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<los>)))
  "Returns full string definition for message of type '<los>"
  (cl:format cl:nil "string robotName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'los)))
  "Returns full string definition for message of type 'los"
  (cl:format cl:nil "string robotName~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <los>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'robotName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <los>))
  "Converts a ROS message object to a list"
  (cl:list 'los
    (cl:cons ':robotName (robotName msg))
))
