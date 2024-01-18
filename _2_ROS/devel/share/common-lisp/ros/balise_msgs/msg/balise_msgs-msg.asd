
(cl:in-package :asdf)

(defsystem "balise_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ArrayPositionPx" :depends-on ("_package_ArrayPositionPx"))
    (:file "_package_ArrayPositionPx" :depends-on ("_package"))
    (:file "ArrayPositionPxWithType" :depends-on ("_package_ArrayPositionPxWithType"))
    (:file "_package_ArrayPositionPxWithType" :depends-on ("_package"))
    (:file "PositionPx" :depends-on ("_package_PositionPx"))
    (:file "_package_PositionPx" :depends-on ("_package"))
    (:file "PositionPxWithType" :depends-on ("_package_PositionPxWithType"))
    (:file "_package_PositionPxWithType" :depends-on ("_package"))
    (:file "Score" :depends-on ("_package_Score"))
    (:file "_package_Score" :depends-on ("_package"))
  ))