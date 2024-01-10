;; Auto-generated. Do not edit!


(when (boundp 'balise_msgs::ArrayPositionPx)
  (if (not (find-package "BALISE_MSGS"))
    (make-package "BALISE_MSGS"))
  (shadow 'ArrayPositionPx (find-package "BALISE_MSGS")))
(unless (find-package "BALISE_MSGS::ARRAYPOSITIONPX")
  (make-package "BALISE_MSGS::ARRAYPOSITIONPX"))

(in-package "ROS")
;;//! \htmlinclude ArrayPositionPx.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass balise_msgs::ArrayPositionPx
  :super ros::object
  :slots (_header _array_of_positionspx ))

(defmethod balise_msgs::ArrayPositionPx
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:array_of_positionspx __array_of_positionspx) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _array_of_positionspx __array_of_positionspx)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
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
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; balise_msgs/PositionPx[] _array_of_positionspx
    (apply #'+ (send-all _array_of_positionspx :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; balise_msgs/PositionPx[] _array_of_positionspx
     (write-long (length _array_of_positionspx) s)
     (dolist (elem _array_of_positionspx)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
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

(setf (get balise_msgs::ArrayPositionPx :md5sum-) "74175f7ab9c85114e7078a9141456233")
(setf (get balise_msgs::ArrayPositionPx :datatype-) "balise_msgs/ArrayPositionPx")
(setf (get balise_msgs::ArrayPositionPx :definition-)
      "Header header
PositionPx[] array_of_positionspx
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: balise_msgs/PositionPx
int32 x
int32 y 
int32 theta

")



(provide :balise_msgs/ArrayPositionPx "74175f7ab9c85114e7078a9141456233")


