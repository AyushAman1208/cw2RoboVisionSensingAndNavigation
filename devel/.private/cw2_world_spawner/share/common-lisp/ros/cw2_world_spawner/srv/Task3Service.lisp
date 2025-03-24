; Auto-generated. Do not edit!


(cl:in-package cw2_world_spawner-srv)


;//! \htmlinclude Task3Service-request.msg.html

(cl:defclass <Task3Service-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Task3Service-request (<Task3Service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Task3Service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Task3Service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<Task3Service-request> is deprecated: use cw2_world_spawner-srv:Task3Service-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Task3Service-request>) ostream)
  "Serializes a message object of type '<Task3Service-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Task3Service-request>) istream)
  "Deserializes a message object of type '<Task3Service-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Task3Service-request>)))
  "Returns string type for a service object of type '<Task3Service-request>"
  "cw2_world_spawner/Task3ServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task3Service-request)))
  "Returns string type for a service object of type 'Task3Service-request"
  "cw2_world_spawner/Task3ServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Task3Service-request>)))
  "Returns md5sum for a message object of type '<Task3Service-request>"
  "a4cdcbf342ebe5d2a91b198b8ea6a615")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Task3Service-request)))
  "Returns md5sum for a message object of type 'Task3Service-request"
  "a4cdcbf342ebe5d2a91b198b8ea6a615")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Task3Service-request>)))
  "Returns full string definition for message of type '<Task3Service-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Task3Service-request)))
  "Returns full string definition for message of type 'Task3Service-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Task3Service-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Task3Service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Task3Service-request
))
;//! \htmlinclude Task3Service-response.msg.html

(cl:defclass <Task3Service-response> (roslisp-msg-protocol:ros-message)
  ((total_num_shapes
    :reader total_num_shapes
    :initarg :total_num_shapes
    :type cl:integer
    :initform 0)
   (num_most_common_shape
    :reader num_most_common_shape
    :initarg :num_most_common_shape
    :type cl:integer
    :initform 0)
   (most_common_shape_vector
    :reader most_common_shape_vector
    :initarg :most_common_shape_vector
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Task3Service-response (<Task3Service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Task3Service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Task3Service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<Task3Service-response> is deprecated: use cw2_world_spawner-srv:Task3Service-response instead.")))

(cl:ensure-generic-function 'total_num_shapes-val :lambda-list '(m))
(cl:defmethod total_num_shapes-val ((m <Task3Service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:total_num_shapes-val is deprecated.  Use cw2_world_spawner-srv:total_num_shapes instead.")
  (total_num_shapes m))

(cl:ensure-generic-function 'num_most_common_shape-val :lambda-list '(m))
(cl:defmethod num_most_common_shape-val ((m <Task3Service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:num_most_common_shape-val is deprecated.  Use cw2_world_spawner-srv:num_most_common_shape instead.")
  (num_most_common_shape m))

(cl:ensure-generic-function 'most_common_shape_vector-val :lambda-list '(m))
(cl:defmethod most_common_shape_vector-val ((m <Task3Service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:most_common_shape_vector-val is deprecated.  Use cw2_world_spawner-srv:most_common_shape_vector instead.")
  (most_common_shape_vector m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Task3Service-response>) ostream)
  "Serializes a message object of type '<Task3Service-response>"
  (cl:let* ((signed (cl:slot-value msg 'total_num_shapes)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_most_common_shape)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'most_common_shape_vector))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'most_common_shape_vector))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Task3Service-response>) istream)
  "Deserializes a message object of type '<Task3Service-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'total_num_shapes) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_most_common_shape) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'most_common_shape_vector) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'most_common_shape_vector)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Task3Service-response>)))
  "Returns string type for a service object of type '<Task3Service-response>"
  "cw2_world_spawner/Task3ServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task3Service-response)))
  "Returns string type for a service object of type 'Task3Service-response"
  "cw2_world_spawner/Task3ServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Task3Service-response>)))
  "Returns md5sum for a message object of type '<Task3Service-response>"
  "a4cdcbf342ebe5d2a91b198b8ea6a615")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Task3Service-response)))
  "Returns md5sum for a message object of type 'Task3Service-response"
  "a4cdcbf342ebe5d2a91b198b8ea6a615")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Task3Service-response>)))
  "Returns full string definition for message of type '<Task3Service-response>"
  (cl:format cl:nil "int64 total_num_shapes~%int64 num_most_common_shape~%int64[] most_common_shape_vector~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Task3Service-response)))
  "Returns full string definition for message of type 'Task3Service-response"
  (cl:format cl:nil "int64 total_num_shapes~%int64 num_most_common_shape~%int64[] most_common_shape_vector~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Task3Service-response>))
  (cl:+ 0
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'most_common_shape_vector) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Task3Service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Task3Service-response
    (cl:cons ':total_num_shapes (total_num_shapes msg))
    (cl:cons ':num_most_common_shape (num_most_common_shape msg))
    (cl:cons ':most_common_shape_vector (most_common_shape_vector msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Task3Service)))
  'Task3Service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Task3Service)))
  'Task3Service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Task3Service)))
  "Returns string type for a service object of type '<Task3Service>"
  "cw2_world_spawner/Task3Service")