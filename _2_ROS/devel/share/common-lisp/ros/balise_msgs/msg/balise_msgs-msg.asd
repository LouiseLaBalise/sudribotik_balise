
(cl:in-package :asdf)

(defsystem "balise_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PositionAruco" :depends-on ("_package_PositionAruco"))
    (:file "_package_PositionAruco" :depends-on ("_package"))
    (:file "PositionGameElements" :depends-on ("_package_PositionGameElements"))
    (:file "_package_PositionGameElements" :depends-on ("_package"))
    (:file "PositionPx" :depends-on ("_package_PositionPx"))
    (:file "_package_PositionPx" :depends-on ("_package"))
    (:file "PositionRobot" :depends-on ("_package_PositionRobot"))
    (:file "_package_PositionRobot" :depends-on ("_package"))
    (:file "Score" :depends-on ("_package_Score"))
    (:file "_package_Score" :depends-on ("_package"))
  ))