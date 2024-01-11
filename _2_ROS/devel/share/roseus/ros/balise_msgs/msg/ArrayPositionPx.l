;; Auto-generated. Do not edit!


(when (boundp 'balise_msgs::ArrayPositionPx)
  (if (not (find-package "BALISE_MSGS"))
    (make-package "BALISE_MSGS"))
  (shadow 'ArrayPositionPx (find-package "BALISE_MSGS")))
(unless (find-package "BALISE_MSGS::ARRAYPOSITIONPX")
  (make-package "BALISE_MSGS::ARRAYPOSITIONPX"))

(in-package "ROS")
;;//! \htmlinclude ArrayPositionPx.msg.html


(defclass balise_msgs::ArrayPositionPx
  :super ros::object
  :slots (_array_of_positionspx ))

(defmethod balise_msgs::ArrayPositionPx
  (:init
   (&key
    ((:array_of_positionspx __array_of_positionspx) ())
    )
   (send-super :init)
   (setq _array_of_positionspx __array_of_positionspx)
   self)
  (:array_of_positionspx
   (&rest __array_of_positionspx)
   (if (keywordp (car __array_of_positionspx))
       (send* _array_of_positionspx __array_of_positionspx)
     (progn
       (if __array_of_positionspx (setq _array_of_positionspx (car __array_of_positionspx)))
       _array_of_positionspx)))
  (:serialization-length
   ()
   (+
    ;; balise_msgs/PositionPx[] _array_of_positionspx
    (apply #'+ (send-all _array_of_positionspx :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; balise_msgs/PositionPx[] _array_of_positionspx
     (write-long (length _array_of_positionspx) s)
     (dolist (elem _array_of_positionspx)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; balise_msgs/PositionPx[] _array_of_positionspx
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _array_of_positionspx (let (r) (dotimes (i n) (push (instance balise_msgs::PositionPx :init) r)) r))
     (dolist (elem- _array_of_positionspx)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get balise_msgs::ArrayPositionPx :md5sum-) "4d42662441de878808f5bfe3b2bdeae4")
(setf (get balise_msgs::ArrayPositionPx :datatype-) "balise_msgs/ArrayPositionPx")
(setf (get balise_msgs::ArrayPositionPx :definition-)
      "PositionPx[] array_of_positionspx
================================================================================
MSG: balise_msgs/PositionPx
int32 x
int32 y 
int32 theta

")



(provide :balise_msgs/ArrayPositionPx "4d42662441de878808f5bfe3b2bdeae4")


