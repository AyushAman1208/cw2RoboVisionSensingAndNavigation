;; Auto-generated. Do not edit!


(when (boundp 'cw2_world_spawner::Task3Service)
  (if (not (find-package "CW2_WORLD_SPAWNER"))
    (make-package "CW2_WORLD_SPAWNER"))
  (shadow 'Task3Service (find-package "CW2_WORLD_SPAWNER")))
(unless (find-package "CW2_WORLD_SPAWNER::TASK3SERVICE")
  (make-package "CW2_WORLD_SPAWNER::TASK3SERVICE"))
(unless (find-package "CW2_WORLD_SPAWNER::TASK3SERVICEREQUEST")
  (make-package "CW2_WORLD_SPAWNER::TASK3SERVICEREQUEST"))
(unless (find-package "CW2_WORLD_SPAWNER::TASK3SERVICERESPONSE")
  (make-package "CW2_WORLD_SPAWNER::TASK3SERVICERESPONSE"))

(in-package "ROS")





(defclass cw2_world_spawner::Task3ServiceRequest
  :super ros::object
  :slots ())

(defmethod cw2_world_spawner::Task3ServiceRequest
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

(defclass cw2_world_spawner::Task3ServiceResponse
  :super ros::object
  :slots (_total_num_shapes _num_most_common_shape _most_common_shape_vector ))

(defmethod cw2_world_spawner::Task3ServiceResponse
  (:init
   (&key
    ((:total_num_shapes __total_num_shapes) 0)
    ((:num_most_common_shape __num_most_common_shape) 0)
    ((:most_common_shape_vector __most_common_shape_vector) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _total_num_shapes (round __total_num_shapes))
   (setq _num_most_common_shape (round __num_most_common_shape))
   (setq _most_common_shape_vector __most_common_shape_vector)
   self)
  (:total_num_shapes
   (&optional __total_num_shapes)
   (if __total_num_shapes (setq _total_num_shapes __total_num_shapes)) _total_num_shapes)
  (:num_most_common_shape
   (&optional __num_most_common_shape)
   (if __num_most_common_shape (setq _num_most_common_shape __num_most_common_shape)) _num_most_common_shape)
  (:most_common_shape_vector
   (&optional __most_common_shape_vector)
   (if __most_common_shape_vector (setq _most_common_shape_vector __most_common_shape_vector)) _most_common_shape_vector)
  (:serialization-length
   ()
   (+
    ;; int64 _total_num_shapes
    8
    ;; int64 _num_most_common_shape
    8
    ;; int64[] _most_common_shape_vector
    (* 8    (length _most_common_shape_vector)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _total_num_shapes
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _total_num_shapes (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _total_num_shapes) (= (length (_total_num_shapes . bv)) 2)) ;; bignum
              (write-long (ash (elt (_total_num_shapes . bv) 0) 0) s)
              (write-long (ash (elt (_total_num_shapes . bv) 1) -1) s))
             ((and (class _total_num_shapes) (= (length (_total_num_shapes . bv)) 1)) ;; big1
              (write-long (elt (_total_num_shapes . bv) 0) s)
              (write-long (if (>= _total_num_shapes 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _total_num_shapes s)(write-long (if (>= _total_num_shapes 0) 0 #xffffffff) s)))
     ;; int64 _num_most_common_shape
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_most_common_shape (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_most_common_shape) (= (length (_num_most_common_shape . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_most_common_shape . bv) 0) 0) s)
              (write-long (ash (elt (_num_most_common_shape . bv) 1) -1) s))
             ((and (class _num_most_common_shape) (= (length (_num_most_common_shape . bv)) 1)) ;; big1
              (write-long (elt (_num_most_common_shape . bv) 0) s)
              (write-long (if (>= _num_most_common_shape 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_most_common_shape s)(write-long (if (>= _num_most_common_shape 0) 0 #xffffffff) s)))
     ;; int64[] _most_common_shape_vector
     (write-long (length _most_common_shape_vector) s)
     (dotimes (i (length _most_common_shape_vector))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _most_common_shape_vector i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _most_common_shape_vector i)) (= (length ((elt _most_common_shape_vector i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _most_common_shape_vector i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _most_common_shape_vector i) . bv) 1) -1) s))
             ((and (class (elt _most_common_shape_vector i)) (= (length ((elt _most_common_shape_vector i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _most_common_shape_vector i) . bv) 0) s)
              (write-long (if (>= (elt _most_common_shape_vector i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _most_common_shape_vector i) s)(write-long (if (>= (elt _most_common_shape_vector i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _total_num_shapes
#+(or :alpha :irix6 :x86_64)
      (setf _total_num_shapes (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _total_num_shapes (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _num_most_common_shape
#+(or :alpha :irix6 :x86_64)
      (setf _num_most_common_shape (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_most_common_shape (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64[] _most_common_shape_vector
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _most_common_shape_vector (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _most_common_shape_vector i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _most_common_shape_vector i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(defclass cw2_world_spawner::Task3Service
  :super ros::object
  :slots ())

(setf (get cw2_world_spawner::Task3Service :md5sum-) "a4cdcbf342ebe5d2a91b198b8ea6a615")
(setf (get cw2_world_spawner::Task3Service :datatype-) "cw2_world_spawner/Task3Service")
(setf (get cw2_world_spawner::Task3Service :request) cw2_world_spawner::Task3ServiceRequest)
(setf (get cw2_world_spawner::Task3Service :response) cw2_world_spawner::Task3ServiceResponse)

(defmethod cw2_world_spawner::Task3ServiceRequest
  (:response () (instance cw2_world_spawner::Task3ServiceResponse :init)))

(setf (get cw2_world_spawner::Task3ServiceRequest :md5sum-) "a4cdcbf342ebe5d2a91b198b8ea6a615")
(setf (get cw2_world_spawner::Task3ServiceRequest :datatype-) "cw2_world_spawner/Task3ServiceRequest")
(setf (get cw2_world_spawner::Task3ServiceRequest :definition-)
      "---
int64 total_num_shapes
int64 num_most_common_shape
int64[] most_common_shape_vector

")

(setf (get cw2_world_spawner::Task3ServiceResponse :md5sum-) "a4cdcbf342ebe5d2a91b198b8ea6a615")
(setf (get cw2_world_spawner::Task3ServiceResponse :datatype-) "cw2_world_spawner/Task3ServiceResponse")
(setf (get cw2_world_spawner::Task3ServiceResponse :definition-)
      "---
int64 total_num_shapes
int64 num_most_common_shape
int64[] most_common_shape_vector

")



(provide :cw2_world_spawner/Task3Service "a4cdcbf342ebe5d2a91b198b8ea6a615")


