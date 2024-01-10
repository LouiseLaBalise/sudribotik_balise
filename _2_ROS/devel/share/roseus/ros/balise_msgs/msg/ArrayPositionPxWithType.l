;; Auto-generated. Do not edit!


(when (boundp 'balise_msgs::ArrayPositionPxWithType)
  (if (not (find-package "BALISE_MSGS"))
    (make-package "BALISE_MSGS"))
  (shadow 'ArrayPositionPxWithType (find-package "BALISE_MSGS")))
(unless (find-package "BALISE_MSGS::ARRAYPOSITIONPXWITHTYPE")
  (make-package "BALISE_MSGS::ARRAYPOSITIONPXWITHTYPE"))

(in-package "ROS")
;;//! \htmlinclude ArrayPositionPxWithType.msg.html


(defclass balise_msgs::ArrayPositionPxWithType
  :super ros::object
  :slots (_array_of_positionspx_with_type ))

(defmethod balise_msgs::ArrayPositionPxWithType
  (:init
   (&key
    ((:array_of_positionspx_with_type __array_of_positionspx_with_type) ())
    )
   (send-super :init)
   (setq _array_of_positionspx_with_type __array_of_positionspx_with_type)
   self)
  (:array_of_positionspx_with_type
   (&rest __array_of_positionspx_with_type)
   (if (keywordp (car __array_of_positionspx_with_type))
       (send* _array_of_positionspx_with_type __array_of_positionspx_with_type)
     (progn
       (if __array_of_positionspx_with_type (setq _array_of_positionspx_with_type (car __array_of_positionspx_with_type)))
       _array_of_positionspx_with_type)))
  (:serialization-length
   ()
   (+
    ;; balise_msgs/PositionPxWithType[] _array_of_positionspx_with_type
    (apply #'+ (send-all _array_of_positionspx_with_type :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; balise_msgs/PositionPxWithType[] _array_of_positionspx_with_type
     (write-long (length _array_of_positionspx_with_type) s)
     (dolist (elem _array_of_positionspx_with_type)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; balise_msgs/PositionPxWithType[] _array_of_positionspx_with_type
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _array_of_positionspx_with_type (let (r) (dotimes (i n) (push (instance balise_msgs::PositionPxWithType :init) r)) r))
     (dolist (elem- _array_of_positionspx_with_type)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get balise_msgs::ArrayPositionPxWithType :md5sum-) "bb0df7150251fed4c0b95568bc63c1f4")
(setf (get balise_msgs::ArrayPositionPxWithType :datatype-) "balise_msgs/ArrayPositionPxWithType")
(setf (get balise_msgs::ArrayPositionPxWithType :definition-)
      "PositionPxWithType[] array_of_positionspx_with_type
================================================================================
MSG: balise_msgs/PositionPxWithType
int32 x
int32 y 
int32 theta
string type
")



(provide :balise_msgs/ArrayPositionPxWithType "bb0df7150251fed4c0b95568bc63c1f4")


