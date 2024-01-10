;; Auto-generated. Do not edit!


(when (boundp 'balise::Obj)
  (if (not (find-package "BALISE"))
    (make-package "BALISE"))
  (shadow 'Obj (find-package "BALISE")))
(unless (find-package "BALISE::OBJ")
  (make-package "BALISE::OBJ"))

(in-package "ROS")
;;//! \htmlinclude Obj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass balise::Obj
  :super ros::object
  :slots (_dscript _ident _position _theta ))

(defmethod balise::Obj
  (:init
   (&key
    ((:dscript __dscript) "")
    ((:ident __ident) 0)
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _dscript (string __dscript))
   (setq _ident (round __ident))
   (setq _position __position)
   (setq _theta (float __theta))
   self)
  (:dscript
   (&optional __dscript)
   (if __dscript (setq _dscript __dscript)) _dscript)
  (:ident
   (&optional __ident)
   (if __ident (setq _ident __ident)) _ident)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; string _dscript
    4 (length _dscript)
    ;; uint32 _ident
    4
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _dscript
       (write-long (length _dscript) s) (princ _dscript s)
     ;; uint32 _ident
       (write-long _ident s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _dscript
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _dscript (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _ident
     (setq _ident (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get balise::Obj :md5sum-) "f95b3051f8929661f923cb09696172fb")
(setf (get balise::Obj :datatype-) "balise/Obj")
(setf (get balise::Obj :definition-)
      "string dscript 
uint32 ident
geometry_msgs/Vector3 position
float64 theta
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :balise/Obj "f95b3051f8929661f923cb09696172fb")


