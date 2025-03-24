; Auto-generated. Do not edit!


(cl:in-package cw2_world_spawner-srv)


;//! \htmlinclude Task2Service-request.msg.html

(cl:defclass <Task2Service-request> (roslisp-msg-protocol:ros-message)
  ((ref_object_points
    :reader ref_object_points
    :initarg :ref_object_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (mystery_object_point
    :reader mystery_object_point
    :initarg :mystery_object_point
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass Task2Service-request (<Task2Service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Task2Service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Task2Service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<Task2Service-request> is deprecated: use cw2_world_spawner-srv:Task2Service-request instead.")))

(cl:ensure-generic-function 'ref_object_points-val :lambda-list '(m))
(cl:defmethod ref_object_points-val ((m <Task2Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:ref_object_points-val is deprecated.  Use cw2_world_spawner-srv:ref_object_points instead.")
  (ref_object_points m))

(cl:ensure-generic-function 'mystery_object_point-val :lambda-list '(m))
(cl:defmethod mystery_object_point-val ((m <Task2Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:mystery_object_point-val is deprecated.  Use cw2_world_spawner-srv:mystery_object_point instead.")
  (mystery_object_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Task2Service-request>) ostream)
  "Serializes a message object of type '<Task2Service-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ref_object_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ref_object_points))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mystery_object_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Task2Service-request>) istream)
  "Deserializes a message object of type '<Task2Service-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ref_object_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ref_object_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mystery_object_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Task2Service-request>)))
  "Returns string type for a service object of type '<Task2Service-request>"
  "cw2_world_spawner/Task2ServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task2Service-request)))
  "Returns string type for a service object of type 'Task2Service-request"
  "cw2_world_spawner/Task2ServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Task2Service-request>)))
  "Returns md5sum for a message object of type '<Task2Service-request>"
  "48fa6a1a8f22948fb54540fdb831dc23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Task2Service-request)))
  "Returns md5sum for a message object of type 'Task2Service-request"
  "48fa6a1a8f22948fb54540fdb831dc23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Task2Service-request>)))
  "Returns full string definition for message of type '<Task2Service-request>"
  (cl:format cl:nil "geometry_msgs/PointStamped[] ref_object_points~%geometry_msgs/PointStamped mystery_object_point~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Task2Service-request)))
  "Returns full string definition for message of type 'Task2Service-request"
  (cl:format cl:nil "geometry_msgs/PointStamped[] ref_object_points~%geometry_msgs/PointStamped mystery_object_point~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Task2Service-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ref_object_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mystery_object_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Task2Service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Task2Service-request
    (cl:cons ':ref_object_points (ref_object_points msg))
    (cl:cons ':mystery_object_point (mystery_object_point msg))
))
;//! \htmlinclude Task2Service-response.msg.html

(cl:defclass <Task2Service-response> (roslisp-msg-protocol:ros-message)
  ((mystery_object_num
    :reader mystery_object_num
    :initarg :mystery_object_num
    :type cl:integer
    :initform 0))
)

(cl:defclass Task2Service-response (<Task2Service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Task2Service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Task2Service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<Task2Service-response> is deprecated: use cw2_world_spawner-srv:Task2Service-response instead.")))

(cl:ensure-generic-function 'mystery_object_num-val :lambda-list '(m))
(cl:defmethod mystery_object_num-val ((m <Task2Service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:mystery_object_num-val is deprecated.  Use cw2_world_spawner-srv:mystery_object_num instead.")
  (mystery_object_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Task2Service-response>) ostream)
  "Serializes a message object of type '<Task2Service-response>"
  (cl:let* ((signed (cl:slot-value msg 'mystery_object_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Task2Service-response>) istream)
  "Deserializes a message object of type '<Task2Service-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mystery_object_num) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Task2Service-response>)))
  "Returns string type for a service object of type '<Task2Service-response>"
  "cw2_world_spawner/Task2ServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task2Service-response)))
  "Returns string type for a service object of type 'Task2Service-response"
  "cw2_world_spawner/Task2ServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Task2Service-response>)))
  "Returns md5sum for a message object of type '<Task2Service-response>"
  "48fa6a1a8f22948fb54540fdb831dc23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Task2Service-response)))
  "Returns md5sum for a message object of type 'Task2Service-response"
  "48fa6a1a8f22948fb54540fdb831dc23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Task2Service-response>)))
  "Returns full string definition for message of type '<Task2Service-response>"
  (cl:format cl:nil "int64 mystery_object_num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Task2Service-response)))
  "Returns full string definition for message of type 'Task2Service-response"
  (cl:format cl:nil "int64 mystery_object_num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Task2Service-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Task2Service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Task2Service-response
    (cl:cons ':mystery_object_num (mystery_object_num msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Task2Service)))
  'Task2Service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Task2Service)))
  'Task2Service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task2Service)))
  "Returns string type for a service object of type '<Task2Service>"
  "cw2_world_spawner/Task2Service")