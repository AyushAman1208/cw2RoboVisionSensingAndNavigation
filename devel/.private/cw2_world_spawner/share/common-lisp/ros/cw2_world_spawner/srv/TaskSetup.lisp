; Auto-generated. Do not edit!


(cl:in-package cw2_world_spawner-srv)


;//! \htmlinclude TaskSetup-request.msg.html

(cl:defclass <TaskSetup-request> (roslisp-msg-protocol:ros-message)
  ((task_index
    :reader task_index
    :initarg :task_index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TaskSetup-request (<TaskSetup-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskSetup-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskSetup-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<TaskSetup-request> is deprecated: use cw2_world_spawner-srv:TaskSetup-request instead.")))

(cl:ensure-generic-function 'task_index-val :lambda-list '(m))
(cl:defmethod task_index-val ((m <TaskSetup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cw2_world_spawner-srv:task_index-val is deprecated.  Use cw2_world_spawner-srv:task_index instead.")
  (task_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskSetup-request>) ostream)
  "Serializes a message object of type '<TaskSetup-request>"
  (cl:let* ((signed (cl:slot-value msg 'task_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskSetup-request>) istream)
  "Deserializes a message object of type '<TaskSetup-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskSetup-request>)))
  "Returns string type for a service object of type '<TaskSetup-request>"
  "cw2_world_spawner/TaskSetupRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskSetup-request)))
  "Returns string type for a service object of type 'TaskSetup-request"
  "cw2_world_spawner/TaskSetupRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskSetup-request>)))
  "Returns md5sum for a message object of type '<TaskSetup-request>"
  "713f50eb6979d6bda1389d9b25c308e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskSetup-request)))
  "Returns md5sum for a message object of type 'TaskSetup-request"
  "713f50eb6979d6bda1389d9b25c308e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskSetup-request>)))
  "Returns full string definition for message of type '<TaskSetup-request>"
  (cl:format cl:nil "int8 task_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskSetup-request)))
  "Returns full string definition for message of type 'TaskSetup-request"
  (cl:format cl:nil "int8 task_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskSetup-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskSetup-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskSetup-request
    (cl:cons ':task_index (task_index msg))
))
;//! \htmlinclude TaskSetup-response.msg.html

(cl:defclass <TaskSetup-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TaskSetup-response (<TaskSetup-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TaskSetup-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TaskSetup-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cw2_world_spawner-srv:<TaskSetup-response> is deprecated: use cw2_world_spawner-srv:TaskSetup-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TaskSetup-response>) ostream)
  "Serializes a message object of type '<TaskSetup-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TaskSetup-response>) istream)
  "Deserializes a message object of type '<TaskSetup-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TaskSetup-response>)))
  "Returns string type for a service object of type '<TaskSetup-response>"
  "cw2_world_spawner/TaskSetupResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskSetup-response)))
  "Returns string type for a service object of type 'TaskSetup-response"
  "cw2_world_spawner/TaskSetupResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TaskSetup-response>)))
  "Returns md5sum for a message object of type '<TaskSetup-response>"
  "713f50eb6979d6bda1389d9b25c308e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TaskSetup-response)))
  "Returns md5sum for a message object of type 'TaskSetup-response"
  "713f50eb6979d6bda1389d9b25c308e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TaskSetup-response>)))
  "Returns full string definition for message of type '<TaskSetup-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TaskSetup-response)))
  "Returns full string definition for message of type 'TaskSetup-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TaskSetup-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TaskSetup-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TaskSetup-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TaskSetup)))
  'TaskSetup-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TaskSetup)))
  'TaskSetup-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TaskSetup)))
  "Returns string type for a service object of type '<TaskSetup>"
  "cw2_world_spawner/TaskSetup")