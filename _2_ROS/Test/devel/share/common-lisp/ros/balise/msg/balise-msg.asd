
(cl:in-package :asdf)

(defsystem "balise-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Obj" :depends-on ("_package_Obj"))
    (:file "_package_Obj" :depends-on ("_package"))
    (:file "ObjArray" :depends-on ("_package_ObjArray"))
    (:file "_package_ObjArray" :depends-on ("_package"))
  ))