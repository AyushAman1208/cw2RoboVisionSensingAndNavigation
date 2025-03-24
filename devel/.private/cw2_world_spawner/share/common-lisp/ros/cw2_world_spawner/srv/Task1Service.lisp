; Auto-generated. Do not edit!


(cl:in-package cw2_world_spawner-srv)


;//! \htmlinclude Task1Service-request.msg.html

(cl:defclass <Task1Service-request> (roslisp-msg-protocol:ros-message)
  ((object_point
    :reader object_point
    :initarg :object_point
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (goal_point
    :reader goal_point
    :initarg :goal_point
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (shape_type
    :reader shape_type
    :initarg :shape_type
    :type cl:string
    :initform ""))
)

(cl:defclass Task1Service-request (<Task1Service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Task1Service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Task1Service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<Task1Service-request> is deprecated: use cw2_world_spawner-srv:Task1Service-request instead.")))

(cl:ensure-generic-function 'object_point-val :lambda-list '(m))
(cl:defmethod object_point-val ((m <Task1Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:object_point-val is deprecated.  Use cw2_world_spawner-srv:object_point instead.")
  (object_point m))

(cl:ensure-generic-function 'goal_point-val :lambda-list '(m))
(cl:defmethod goal_point-val ((m <Task1Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:goal_point-val is deprecated.  Use cw2_world_spawner-srv:goal_point instead.")
  (goal_point m))

(cl:ensure-generic-function 'shape_type-val :lambda-list '(m))
(cl:defmethod shape_type-val ((m <Task1Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:shape_type-val is deprecated.  Use cw2_world_spawner-srv:shape_type instead.")
  (shape_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Task1Service-request>) ostream)
  "Serializes a message object of type '<Task1Service-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_point) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'shape_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'shape_type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Task1Service-request>) istream)
  "Deserializes a message object of type '<Task1Service-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_point) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'shape_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'shape_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Task1Service-request>)))
  "Returns string type for a service object of type '<Task1Service-request>"
  "cw2_world_spawner/Task1ServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task1Service-request)))
  "Returns string type for a service object of type 'Task1Service-request"
  "cw2_world_spawner/Task1ServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Task1Service-request>)))
  "Returns md5sum for a message object of type '<Task1Service-request>"
  "ee66ea1bcc1704a8551b6f7b84e9cf9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Task1Service-request)))
  "Returns md5sum for a message object of type 'Task1Service-request"
  "ee66ea1bcc1704a8551b6f7b84e9cf9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Task1Service-request>)))
  "Returns full string definition for message of type '<Task1Service-request>"
  (cl:format cl:nil "geometry_msgs/PointStamped object_point~%geometry_msgs/PointStamped goal_point~%string shape_type~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Task1Service-request)))
  "Returns full string definition for message of type 'Task1Service-request"
  (cl:format cl:nil "geometry_msgs/PointStamped object_point~%geometry_msgs/PointStamped goal_point~%string shape_type~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Task1Service-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_point))
     4 (cl:length (cl:slot-value msg 'shape_type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Task1Service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Task1Service-request
    (cl:cons ':object_point (object_point msg))
    (cl:cons ':goal_point (goal_point msg))
    (cl:cons ':shape_type (shape_type msg))
))
;//! \htmlinclude Task1Service-response.msg.html

(cl:defclass <Task1Service-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Task1Service-response (<Task1Service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Task1Service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Task1Service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<Task1Service-response> is deprecated: use cw2_world_spawner-srv:Task1Service-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Task1Service-response>) ostream)
  "Serializes a message object of type '<Task1Service-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Task1Service-response>) istream)
  "Deserializes a message object of type '<Task1Service-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Task1Service-response>)))
  "Returns string type for a service object of type '<Task1Service-response>"
  "cw2_world_spawner/Task1ServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task1Service-response)))
  "Returns string type for a service object of type 'Task1Service-response"
  "cw2_world_spawner/Task1ServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Task1Service-response>)))
  "Returns md5sum for a message object of type '<Task1Service-response>"
  "ee66ea1bcc1704a8551b6f7b84e9cf9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Task1Service-response)))
  "Returns md5sum for a message object of type 'Task1Service-response"
  "ee66ea1bcc1704a8551b6f7b84e9cf9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Task1Service-response>)))
  "Returns full string definition for message of type '<Task1Service-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Task1Service-response)))
  "Returns full string definition for message of type 'Task1Service-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Task1Service-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Task1Service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Task1Service-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Task1Service)))
  'Task1Service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Task1Service)))
  'Task1Service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task1Service)))
  "Returns string type for a service object of type '<Task1Service>"
  "cw2_world_spawner/Task1Service")