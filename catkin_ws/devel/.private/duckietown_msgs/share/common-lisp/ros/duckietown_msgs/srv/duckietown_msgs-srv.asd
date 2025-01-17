
(cl:in-package :asdf)

(defsystem "duckietown_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GetVariable" :depends-on ("_package_GetVariable"))
    (:file "_package_GetVariable" :depends-on ("_package"))
    (:file "IMUstatus" :depends-on ("_package_IMUstatus"))
    (:file "_package_IMUstatus" :depends-on ("_package"))
    (:file "LFstatus" :depends-on ("_package_LFstatus"))
    (:file "_package_LFstatus" :depends-on ("_package"))
    (:file "SensorsStatus" :depends-on ("_package_SensorsStatus"))
    (:file "_package_SensorsStatus" :depends-on ("_package"))
    (:file "SetFSMState" :depends-on ("_package_SetFSMState"))
    (:file "_package_SetFSMState" :depends-on ("_package"))
    (:file "SetValue" :depends-on ("_package_SetValue"))
    (:file "_package_SetValue" :depends-on ("_package"))
    (:file "SetVariable" :depends-on ("_package_SetVariable"))
    (:file "_package_SetVariable" :depends-on ("_package"))
    (:file "ToFstatus" :depends-on ("_package_ToFstatus"))
    (:file "_package_ToFstatus" :depends-on ("_package"))
  ))