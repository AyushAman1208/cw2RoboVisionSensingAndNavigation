;; Auto-generated. Do not edit!


(when (boundp 'cw2_world_spawner::Task1Service)
  (if (not (find-package "CW2_WORLD_SPAWNER"))
    (make-package "CW2_WORLD_SPAWNER"))
  (shadow 'Task1Service (find-package "CW2_WORLD_SPAWNER")))
(unless (find-package "CW2_WORLD_SPAWNER::TASK1SERVICE")
  (make-package "CW2_WORLD_SPAWNER::TASK1SERVICE"))
(unless (find-package "CW2_WORLD_SPAWNER::TASK1SERVICEREQUEST")
  (make-package "CW2_WORLD_SPAWNER::TASK1SERVICEREQUEST"))
(unless (find-package "CW2_WORLD_SPAWNER::TASK1SERVICERESPONSE")
  (make-package "CW2_WORLD_SPAWNER::TASK1SERVICERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass cw2_world_spawner::Task1ServiceRequest
  :super ros::object
  :slots (_object_point _goal_point _shape_type ))

(defmethod cw2_world_spawner::Task1ServiceRequest
  (:init
   (&key
    ((:object_point __object_point) (instance geometry_msgs::PointStamped :init))
    ((:goal_point __goal_point) (instance geometry_msgs::PointStamped :init))
    ((:shape_type __shape_type) "")
    )
   (send-super :init)
   (setq _object_point __object_point)
   (setq _goal_point __goal_point)
   (setq _shape_type (string __shape_type))
   self)
  (:object_point
   (&rest __object_point)
   (if (keywordp (car __object_point))
       (send* _object_point __object_point)
     (progn
       (if __object_point (setq _object_point (car __object_point)))
       _object_point)))
  (:goal_point
   (&rest __goal_point)
   (if (keywordp (car __goal_point))
       (send* _goal_point __goal_point)
     (progn
       (if __goal_point (setq _goal_point (car __goal_point)))
       _goal_point)))
  (:shape_type
   (&optional __shape_type)
   (if __shape_type (setq _shape_type __shape_type)) _shape_type)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped _object_point
    (send _object_point :serialization-length)
    ;; geometry_msgs/PointStamped _goal_point
    (send _goal_point :serialization-length)
    ;; string _shape_type
    4 (length _shape_type)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped _object_point
       (send _object_point :serialize s)
     ;; geometry_msgs/PointStamped _goal_point
       (send _goal_point :serialize s)
     ;; string _shape_type
       (write-long (length _shape_type) s) (princ _shape_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped _object_point
     (send _object_point :deserialize buf ptr-) (incf ptr- (send _object_point :serialization-length))
   ;; geometry_msgs/PointStamped _goal_point
     (send _goal_point :deserialize buf ptr-) (incf ptr- (send _goal_point :serialization-length))
   ;; string _shape_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _shape_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cw2_world_spawner::Task1ServiceResponse
  :super ros::object
  :slots ())

(defmethod cw2_world_spawner::Task1ServiceResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass cw2_world_spawner::Task1Service
  :super ros::object
  :slots ())

(setf (get cw2_world_spawner::Task1Service :md5sum-) "ee66ea1bcc1704a8551b6f7b84e9cf9e")
(setf (get cw2_world_spawner::Task1Service :datatype-) "cw2_world_spawner/Task1Service")
(setf (get cw2_world_spawner::Task1Service :request) cw2_world_spawner::Task1ServiceRequest)
(setf (get cw2_world_spawner::Task1Service :response) cw2_world_spawner::Task1ServiceResponse)

(defmethod cw2_world_spawner::Task1ServiceRequest
  (:response () (instance cw2_world_spawner::Task1ServiceResponse :init)))

(setf (get cw2_world_spawner::Task1ServiceRequest :md5sum-) "ee66ea1bcc1704a8551b6f7b84e9cf9e")
(setf (get cw2_world_spawner::Task1ServiceRequest :datatype-) "cw2_world_spawner/Task1ServiceRequest")
(setf (get cw2_world_spawner::Task1ServiceRequest :definition-)
      "geometry_msgs/PointStamped object_point
geometry_msgs/PointStamped goal_point
string shape_type

================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---

")

(setf (get cw2_world_spawner::Task1ServiceResponse :md5sum-) "ee66ea1bcc1704a8551b6f7b84e9cf9e")
(setf (get cw2_world_spawner::Task1ServiceResponse :datatype-) "cw2_world_spawner/Task1ServiceResponse")
(setf (get cw2_world_spawner::Task1ServiceResponse :definition-)
      "geometry_msgs/PointStamped object_point
geometry_msgs/PointStamped goal_point
string shape_type

================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---

")



(provide :cw2_world_spawner/Task1Service "ee66ea1bcc1704a8551b6f7b84e9cf9e")


