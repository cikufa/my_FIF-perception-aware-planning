; Auto-generated. Do not edit!


(cl:in-package act_map_exp-srv)


;//! \htmlinclude PlanConfig-request.msg.html

(cl:defclass <PlanConfig-request> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type cl:string
    :initform ""))
)

(cl:defclass PlanConfig-request (<PlanConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name act_map_exp-srv:<PlanConfig-request> is deprecated: use act_map_exp-srv:PlanConfig-request instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <PlanConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader act_map_exp-srv:config-val is deprecated.  Use act_map_exp-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanConfig-request>) ostream)
  "Serializes a message object of type '<PlanConfig-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'config))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'config))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanConfig-request>) istream)
  "Deserializes a message object of type '<PlanConfig-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'config) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'config) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanConfig-request>)))
  "Returns string type for a service object of type '<PlanConfig-request>"
  "act_map_exp/PlanConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanConfig-request)))
  "Returns string type for a service object of type 'PlanConfig-request"
  "act_map_exp/PlanConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanConfig-request>)))
  "Returns md5sum for a message object of type '<PlanConfig-request>"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanConfig-request)))
  "Returns md5sum for a message object of type 'PlanConfig-request"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanConfig-request>)))
  "Returns full string definition for message of type '<PlanConfig-request>"
  (cl:format cl:nil "string config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanConfig-request)))
  "Returns full string definition for message of type 'PlanConfig-request"
  (cl:format cl:nil "string config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanConfig-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanConfig-request
    (cl:cons ':config (config msg))
))
;//! \htmlinclude PlanConfig-response.msg.html

(cl:defclass <PlanConfig-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PlanConfig-response (<PlanConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name act_map_exp-srv:<PlanConfig-response> is deprecated: use act_map_exp-srv:PlanConfig-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanConfig-response>) ostream)
  "Serializes a message object of type '<PlanConfig-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanConfig-response>) istream)
  "Deserializes a message object of type '<PlanConfig-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanConfig-response>)))
  "Returns string type for a service object of type '<PlanConfig-response>"
  "act_map_exp/PlanConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanConfig-response)))
  "Returns string type for a service object of type 'PlanConfig-response"
  "act_map_exp/PlanConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanConfig-response>)))
  "Returns md5sum for a message object of type '<PlanConfig-response>"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanConfig-response)))
  "Returns md5sum for a message object of type 'PlanConfig-response"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanConfig-response>)))
  "Returns full string definition for message of type '<PlanConfig-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanConfig-response)))
  "Returns full string definition for message of type 'PlanConfig-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanConfig-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanConfig-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanConfig)))
  'PlanConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanConfig)))
  'PlanConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanConfig)))
  "Returns string type for a service object of type '<PlanConfig>"
  "act_map_exp/PlanConfig")