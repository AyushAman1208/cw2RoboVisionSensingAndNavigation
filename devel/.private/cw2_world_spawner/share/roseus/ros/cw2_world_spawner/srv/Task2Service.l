;; Auto-generated. Do not edit!


(when (boundp 'cw2_world_spawner::Task2Service)
  (if (not (find-package "CW2_WORLD_SPAWNER"))
    (make-package "CW2_WORLD_SPAWNER"))
  (shadow 'Task2Service (find-package "CW2_WORLD_SPAWNER")))
(unless (find-package "CW2_WORLD_SPAWNER::TASK2SERVICE")
  (make-package "CW2_WORLD_SPAWNER::TASK2SERVICE"))
(unless (find-package "CW2_WORLD_SPAWNER::TASK2SERVICEREQUEST")
  (make-package "CW2_WORLD_SPAWNER::TASK2SERVICEREQUEST"))
(unless (find-package "CW2_WORLD_SPAWNER::TASK2SERVICERESPONSE")
  (make-package "CW2_WORLD_SPAWNER::TASK2SERVICERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass cw2_world_spawner::Task2ServiceRequest
  :super ros::object
  :slots (_ref_object_points _mystery_object_point ))

(defmethod cw2_world_spawner::Task2ServiceRequest
  (:init
   (&key
    ((:ref_object_points __ref_object_points) ())
    ((:mystery_object_point __mystery_object_point) (instance geometry_msgs::PointStamped :init))
    )
   (send-super :init)
   (setq _ref_object_points __ref_object_points)
   (setq _mystery_object_point __mystery_object_point)
   self)
  (:ref_object_points
   (&rest __ref_object_points)
   (if (keywordp (car __ref_object_points))
       (send* _ref_object_points __ref_object_points)
     (progn
       (if __ref_object_points (setq _ref_object_points (car __ref_object_points)))
       _ref_object_points)))
  (:mystery_object_point
   (&rest __mystery_object_point)
   (if (keywordp (car __mystery_object_point))
       (send* _mystery_object_point __mystery_object_point)
     (progn
       (if __mystery_object_point (setq _mystery_object_point (car __mystery_object_point)))
       _mystery_object_point)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PointStamped[] _ref_object_points
    (apply #'+ (send-all _ref_object_points :serialization-length)) 4
    ;; geometry_msgs/PointStamped _mystery_object_point
    (send _mystery_object_point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PointStamped[] _ref_object_points
     (write-long (length _ref_object_points) s)
     (dolist (elem _ref_object_points)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PointStamped _mystery_object_point
       (send _mystery_object_point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PointStamped[] _ref_object_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ref_object_points (let (r) (dotimes (i n) (push (instance geometry_msgs::PointStamped :init) r)) r))
     (dolist (elem- _ref_object_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PointStamped _mystery_object_point
     (send _mystery_object_point :deserialize buf ptr-) (incf ptr- (send _mystery_object_point :serialization-length))
   ;;
   self)
  )

(defclass cw2_world_spawner::Task2ServiceResponse
  :super ros::object
  :slots (_mystery_object_num ))

(defmethod cw2_world_spawner::Task2ServiceResponse
  (:init
   (&key
    ((:mystery_object_num __mystery_object_num) 0)
    )
   (send-super :init)
   (setq _mystery_object_num (round __mystery_object_num))
   self)
  (:mystery_object_num
   (&optional __mystery_object_num)
   (if __mystery_object_num (setq _mystery_object_num __mystery_object_num)) _mystery_object_num)
  (:serialization-length
   ()
   (+
    ;; int64 _mystery_object_num
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _mystery_object_num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _mystery_object_num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _mystery_object_num) (= (length (_mystery_object_num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_mystery_object_num . bv) 0) 0) s)
              (write-long (ash (elt (_mystery_object_num . bv) 1) -1) s))
             ((and (class _mystery_object_num) (= (length (_mystery_object_num . bv)) 1)) ;; big1
              (write-long (elt (_mystery_object_num . bv) 0) s)
              (write-long (if (>= _mystery_object_num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _mystery_object_num s)(write-long (if (>= _mystery_object_num 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _mystery_object_num
#+(or :alpha :irix6 :x86_64)
      (setf _mystery_object_num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _mystery_object_num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass cw2_world_spawner::Task2Service
  :super ros::object
  :slots ())

(setf (get cw2_world_spawner::Task2Service :md5sum-) "48fa6a1a8f22948fb54540fdb831dc23")
(setf (get cw2_world_spawner::Task2Service :datatype-) "cw2_world_spawner/Task2Service")
(setf (get cw2_world_spawner::Task2Service :request) cw2_world_spawner::Task2ServiceRequest)
(setf (get cw2_world_spawner::Task2Service :response) cw2_world_spawner::Task2ServiceResponse)

(defmethod cw2_world_spawner::Task2ServiceRequest
  (:response () (instance cw2_world_spawner::Task2ServiceResponse :init)))

(setf (get cw2_world_spawner::Task2ServiceRequest :md5sum-) "48fa6a1a8f22948fb54540fdb831dc23")
(setf (get cw2_world_spawner::Task2ServiceRequest :datatype-) "cw2_world_spawner/Task2ServiceRequest")
(setf (get cw2_world_spawner::Task2ServiceRequest :definition-)
      "geometry_msgs/PointStamped[] ref_object_points
geometry_msgs/PointStamped mystery_object_point

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
int64 mystery_object_num

")

(setf (get cw2_world_spawner::Task2ServiceResponse :md5sum-) "48fa6a1a8f22948fb54540fdb831dc23")
(setf (get cw2_world_spawner::Task2ServiceResponse :datatype-) "cw2_world_spawner/Task2ServiceResponse")
(setf (get cw2_world_spawner::Task2ServiceResponse :definition-)
      "geometry_msgs/PointStamped[] ref_object_points
geometry_msgs/PointStamped mystery_object_point

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
int64 mystery_object_num

")



(provide :cw2_world_spawner/Task2Service "48fa6a1a8f22948fb54540fdb831dc23")


