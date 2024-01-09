;; Auto-generated. Do not edit!


(when (boundp 'balise_msgs::PositionRobot)
  (if (not (find-package "BALISE_MSGS"))
    (make-package "BALISE_MSGS"))
  (shadow 'PositionRobot (find-package "BALISE_MSGS")))
(unless (find-package "BALISE_MSGS::POSITIONROBOT")
  (make-package "BALISE_MSGS::POSITIONROBOT"))

(in-package "ROS")
;;//! \htmlinclude PositionRobot.msg.html


(defclass balise_msgs::PositionRobot
  :super ros::object
  :slots (_x _y _theta _team _type ))

(defmethod balise_msgs::PositionRobot
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:theta __theta) 0)
    ((:team __team) 0)
    ((:type __type) "")
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _theta (round __theta))
   (setq _team (round __team))
   (setq _type (string __type))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:team
   (&optional __team)
   (if __team (setq _team __team)) _team)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; int32 _theta
    4
    ;; int8 _team
    1
    ;; string _type
    4 (length _type)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; int32 _theta
       (write-long _theta s)
     ;; int8 _team
       (write-byte _team s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _theta
     (setq _theta (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _team
     (setq _team (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _team 127) (setq _team (- _team 256)))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get balise_msgs::PositionRobot :md5sum-) "eb86d22bd0412302c40268f48fe779b4")
(setf (get balise_msgs::PositionRobot :datatype-) "balise_msgs/PositionRobot")
(setf (get balise_msgs::PositionRobot :definition-)
      "int32 x
int32 y 
int32 theta
int8 team
string type
")



(provide :balise_msgs/PositionRobot "eb86d22bd0412302c40268f48fe779b4")


