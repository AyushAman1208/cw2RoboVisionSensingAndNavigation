; Auto-generated. Do not edit!


(cl:in-package cw2_team_x-srv)


;//! \htmlinclude example-request.msg.html

(cl:defclass <example-request> (roslisp-msg-protocol:ros-message)
  ((example_float
    :reader example_float
    :initarg :example_float
    :type cl:float
    :initform 0.0)
   (example_int
    :reader example_int
    :initarg :example_int
    :type cl:integer
    :initform 0)
   (example_string
    :reader example_string
    :initarg :example_string
    :type cl:string
    :initform "")
   (example_pose
    :reader example_pose
    :initarg :example_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (example_std_vector_of_poses
    :reader example_std_vector_of_poses
    :initarg :example_std_vector_of_poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass example-request (<example-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <example-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'example-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_team_x-srv:<example-request> is deprecated: use cw2_team_x-srv:example-request instead.")))

(cl:ensure-generic-function 'example_float-val :lambda-list '(m))
(cl:defmethod example_float-val ((m <example-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_team_x-srv:example_float-val is deprecated.  Use cw2_team_x-srv:example_float instead.")
  (example_float m))

(cl:ensure-generic-function 'example_int-val :lambda-list '(m))
(cl:defmethod example_int-val ((m <example-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_team_x-srv:example_int-val is deprecated.  Use cw2_team_x-srv:example_int instead.")
  (example_int m))

(cl:ensure-generic-function 'example_string-val :lambda-list '(m))
(cl:defmethod example_string-val ((m <example-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_team_x-srv:example_string-val is deprecated.  Use cw2_team_x-srv:example_string instead.")
  (example_string m))

(cl:ensure-generic-function 'example_pose-val :lambda-list '(m))
(cl:defmethod example_pose-val ((m <example-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_team_x-srv:example_pose-val is deprecated.  Use cw2_team_x-srv:example_pose instead.")
  (example_pose m))

(cl:ensure-generic-function 'example_std_vector_of_poses-val :lambda-list '(m))
(cl:defmethod example_std_vector_of_poses-val ((m <example-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_team_x-srv:example_std_vector_of_poses-val is deprecated.  Use cw2_team_x-srv:example_std_vector_of_poses instead.")
  (example_std_vector_of_poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <example-request>) ostream)
  "Serializes a message object of type '<example-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'example_float))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'example_int)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'example_string))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'example_string))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'example_pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'example_std_vector_of_poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'example_std_vector_of_poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <example-request>) istream)
  "Deserializes a message object of type '<example-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'example_float) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'example_int) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'example_string) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'example_string) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'example_pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'example_std_vector_of_poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'example_std_vector_of_poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<example-request>)))
  "Returns string type for a service object of type '<example-request>"
  "cw2_team_x/exampleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'example-request)))
  "Returns string type for a service object of type 'example-request"
  "cw2_team_x/exampleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<example-request>)))
  "Returns md5sum for a message object of type '<example-request>"
  "978efc3b4d217b6a9fca06d03983e5b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'example-request)))
  "Returns md5sum for a message object of type 'example-request"
  "978efc3b4d217b6a9fca06d03983e5b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<example-request>)))
  "Returns full string definition for message of type '<example-request>"
  (cl:format cl:nil "# this is the request message~%float64 example_float~%int32 example_int~%string example_string~%geometry_msgs/Pose example_pose~%geometry_msgs/Pose[] example_std_vector_of_poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'example-request)))
  "Returns full string definition for message of type 'example-request"
  (cl:format cl:nil "# this is the request message~%float64 example_float~%int32 example_int~%string example_string~%geometry_msgs/Pose example_pose~%geometry_msgs/Pose[] example_std_vector_of_poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <example-request>))
  (cl:+ 0
     8
     4
     4 (cl:length (cl:slot-value msg 'example_string))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'example_pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'example_std_vector_of_poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <example-request>))
  "Converts a ROS message object to a list"
  (cl:list 'example-request
    (cl:cons ':example_float (example_float msg))
    (cl:cons ':example_int (example_int msg))
    (cl:cons ':example_string (example_string msg))
    (cl:cons ':example_pose (example_pose msg))
    (cl:cons ':example_std_vector_of_poses (example_std_vector_of_poses msg))
))
;//! \htmlinclude example-response.msg.html

(cl:defclass <example-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (example_std_vector_of_points
    :reader example_std_vector_of_points
    :initarg :example_std_vector_of_points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass example-response (<example-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <example-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'example-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_team_x-srv:<example-response> is deprecated: use cw2_team_x-srv:example-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <example-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_team_x-srv:success-val is deprecated.  Use cw2_team_x-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'example_std_vector_of_points-val :lambda-list '(m))
(cl:defmethod example_std_vector_of_points-val ((m <example-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_team_x-srv:example_std_vector_of_points-val is deprecated.  Use cw2_team_x-srv:example_std_vector_of_points instead.")
  (example_std_vector_of_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <example-response>) ostream)
  "Serializes a message object of type '<example-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'example_std_vector_of_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'example_std_vector_of_points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <example-response>) istream)
  "Deserializes a message object of type '<example-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'example_std_vector_of_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'example_std_vector_of_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<example-response>)))
  "Returns string type for a service object of type '<example-response>"
  "cw2_team_x/exampleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'example-response)))
  "Returns string type for a service object of type 'example-response"
  "cw2_team_x/exampleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<example-response>)))
  "Returns md5sum for a message object of type '<example-response>"
  "978efc3b4d217b6a9fca06d03983e5b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'example-response)))
  "Returns md5sum for a message object of type 'example-response"
  "978efc3b4d217b6a9fca06d03983e5b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<example-response>)))
  "Returns full string definition for message of type '<example-response>"
  (cl:format cl:nil "# this is the return message~%bool success~%geometry_msgs/Point[] example_std_vector_of_points~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'example-response)))
  "Returns full string definition for message of type 'example-response"
  (cl:format cl:nil "# this is the return message~%bool success~%geometry_msgs/Point[] example_std_vector_of_points~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <example-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'example_std_vector_of_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <example-response>))
  "Converts a ROS message object to a list"
  (cl:list 'example-response
    (cl:cons ':success (success msg))
    (cl:cons ':example_std_vector_of_points (example_std_vector_of_points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'example)))
  'example-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'example)))
  'example-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'example)))
  "Returns string type for a service object of type '<example>"
  "cw2_team_x/example")