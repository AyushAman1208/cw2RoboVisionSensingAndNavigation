
(cl:in-package :asdf)

(defsystem "cw2_world_spawner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Task1Service" :depends-on ("_package_Task1Service"))
    (:file "_package_Task1Service" :depends-on ("_package"))
    (:file "Task2Service" :depends-on ("_package_Task2Service"))
    (:file "_package_Task2Service" :depends-on ("_package"))
    (:file "Task3Service" :depends-on ("_package_Task3Service"))
    (:file "_package_Task3Service" :depends-on ("_package"))
    (:file "TaskSetup" :depends-on ("_package_TaskSetup"))
    (:file "_package_TaskSetup" :depends-on ("_package"))
  ))