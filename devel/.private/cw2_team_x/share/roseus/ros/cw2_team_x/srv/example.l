;; Auto-generated. Do not edit!


(when (boundp 'cw2_team_x::example)
  (if (not (find-package "CW2_TEAM_X"))
    (make-package "CW2_TEAM_X"))
  (shadow 'example (find-package "CW2_TEAM_X")))
(unless (find-package "CW2_TEAM_X::EXAMPLE")
  (make-package "CW2_TEAM_X::EXAMPLE"))
(unless (find-package "CW2_TEAM_X::EXAMPLEREQUEST")
  (make-package "CW2_TEAM_X::EXAMPLEREQUEST"))
(unless (find-package "CW2_TEAM_X::EXAMPLERESPONSE")
  (make-package "CW2_TEAM_X::EXAMPLERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass cw2_team_x::exampleRequest
  :super ros::object
  :slots (_example_float _example_int _example_string _example_pose _example_std_vector_of_poses ))

(defmethod cw2_team_x::exampleRequest
  (:init
   (&key
    ((:example_float __example_float) 0.0)
    ((:example_int __example_int) 0)
    ((:example_string __example_string) "")
    ((:example_pose __example_pose) (instance geometry_msgs::Pose :init))
    ((:example_std_vector_of_poses __example_std_vector_of_poses) ())
    )
   (send-super :init)
   (setq _example_float (float __example_float))
   (setq _example_int (round __example_int))
   (setq _example_string (string __example_string))
   (setq _example_pose __example_pose)
   (setq _example_std_vector_of_poses __example_std_vector_of_poses)
   self)
  (:example_float
   (&optional __example_float)
   (if __example_float (setq _example_float __example_float)) _example_float)
  (:example_int
   (&optional __example_int)
   (if __example_int (setq _example_int __example_int)) _example_int)
  (:example_string
   (&optional __example_string)
   (if __example_string (setq _example_string __example_string)) _example_string)
  (:example_pose
   (&rest __example_pose)
   (if (keywordp (car __example_pose))
       (send* _example_pose __example_pose)
     (progn
       (if __example_pose (setq _example_pose (car __example_pose)))
       _example_pose)))
  (:example_std_vector_of_poses
   (&rest __example_std_vector_of_poses)
   (if (keywordp (car __example_std_vector_of_poses))
       (send* _example_std_vector_of_poses __example_std_vector_of_poses)
     (progn
       (if __example_std_vector_of_poses (setq _example_std_vector_of_poses (car __example_std_vector_of_poses)))
       _example_std_vector_of_poses)))
  (:serialization-length
   ()
   (+
    ;; float64 _example_float
    8
    ;; int32 _example_int
    4
    ;; string _example_string
    4 (length _example_string)
    ;; geometry_msgs/Pose _example_pose
    (send _example_pose :serialization-length)
    ;; geometry_msgs/Pose[] _example_std_vector_of_poses
    (apply #'+ (send-all _example_std_vector_of_poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _example_float
       (sys::poke _example_float (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _example_int
       (write-long _example_int s)
     ;; string _example_string
       (write-long (length _example_string) s) (princ _example_string s)
     ;; geometry_msgs/Pose _example_pose
       (send _example_pose :serialize s)
     ;; geometry_msgs/Pose[] _example_std_vector_of_poses
     (write-long (length _example_std_vector_of_poses) s)
     (dolist (elem _example_std_vector_of_poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _example_float
     (setq _example_float (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _example_int
     (setq _example_int (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _example_string
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _example_string (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _example_pose
     (send _example_pose :deserialize buf ptr-) (incf ptr- (send _example_pose :serialization-length))
   ;; geometry_msgs/Pose[] _example_std_vector_of_poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _example_std_vector_of_poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _example_std_vector_of_poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass cw2_team_x::exampleResponse
  :super ros::object
  :slots (_success _example_std_vector_of_points ))

(defmethod cw2_team_x::exampleResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:example_std_vector_of_points __example_std_vector_of_points) ())
    )
   (send-super :init)
   (setq _success __success)
   (setq _example_std_vector_of_points __example_std_vector_of_points)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:example_std_vector_of_points
   (&rest __example_std_vector_of_points)
   (if (keywordp (car __example_std_vector_of_points))
       (send* _example_std_vector_of_points __example_std_vector_of_points)
     (progn
       (if __example_std_vector_of_points (setq _example_std_vector_of_points (car __example_std_vector_of_points)))
       _example_std_vector_of_points)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; geometry_msgs/Point[] _example_std_vector_of_points
    (apply #'+ (send-all _example_std_vector_of_points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point[] _example_std_vector_of_points
     (write-long (length _example_std_vector_of_points) s)
     (dolist (elem _example_std_vector_of_points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point[] _example_std_vector_of_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _example_std_vector_of_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _example_std_vector_of_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass cw2_team_x::example
  :super ros::object
  :slots ())

(setf (get cw2_team_x::example :md5sum-) "978efc3b4d217b6a9fca06d03983e5b2")
(setf (get cw2_team_x::example :datatype-) "cw2_team_x/example")
(setf (get cw2_team_x::example :request) cw2_team_x::exampleRequest)
(setf (get cw2_team_x::example :response) cw2_team_x::exampleResponse)

(defmethod cw2_team_x::exampleRequest
  (:response () (instance cw2_team_x::exampleResponse :init)))

(setf (get cw2_team_x::exampleRequest :md5sum-) "978efc3b4d217b6a9fca06d03983e5b2")
(setf (get cw2_team_x::exampleRequest :datatype-) "cw2_team_x/exampleRequest")
(setf (get cw2_team_x::exampleRequest :definition-)
      "# this is the request message
float64 example_float
int32 example_int
string example_string
geometry_msgs/Pose example_pose
geometry_msgs/Pose[] example_std_vector_of_poses

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
# this is the return message
bool success
geometry_msgs/Point[] example_std_vector_of_points

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get cw2_team_x::exampleResponse :md5sum-) "978efc3b4d217b6a9fca06d03983e5b2")
(setf (get cw2_team_x::exampleResponse :datatype-) "cw2_team_x/exampleResponse")
(setf (get cw2_team_x::exampleResponse :definition-)
      "# this is the request message
float64 example_float
int32 example_int
string example_string
geometry_msgs/Pose example_pose
geometry_msgs/Pose[] example_std_vector_of_poses

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
# this is the return message
bool success
geometry_msgs/Point[] example_std_vector_of_points

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :cw2_team_x/example "978efc3b4d217b6a9fca06d03983e5b2")


