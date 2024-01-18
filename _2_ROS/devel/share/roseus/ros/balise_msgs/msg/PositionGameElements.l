;; Auto-generated. Do not edit!


(when (boundp 'balise_msgs::PositionGameElements)
  (if (not (find-package "BALISE_MSGS"))
    (make-package "BALISE_MSGS"))
  (shadow 'PositionGameElements (find-package "BALISE_MSGS")))
(unless (find-package "BALISE_MSGS::POSITIONGAMEELEMENTS")
  (make-package "BALISE_MSGS::POSITIONGAMEELEMENTS"))

(in-package "ROS")
;;//! \htmlinclude PositionGameElements.msg.html


(defclass balise_msgs::PositionGameElements
  :super ros::object
  :slots (_x _y _theta _type ))

(defmethod balise_msgs::PositionGameElements
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:theta __theta) 0)
    ((:type __type) "")
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _theta (round __theta))
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
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get balise_msgs::PositionGameElements :md5sum-) "18f807f0bff0003047e6e888eca3bfa5")
(setf (get balise_msgs::PositionGameElements :datatype-) "balise_msgs/PositionGameElements")
(setf (get balise_msgs::PositionGameElements :definition-)
      "int32 x
int32 y 
int32 theta
string type
")



(provide :balise_msgs/PositionGameElements "18f807f0bff0003047e6e888eca3bfa5")


