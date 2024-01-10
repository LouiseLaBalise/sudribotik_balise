;; Auto-generated. Do not edit!


(when (boundp 'balise::ObjArray)
  (if (not (find-package "BALISE"))
    (make-package "BALISE"))
  (shadow 'ObjArray (find-package "BALISE")))
(unless (find-package "BALISE::OBJARRAY")
  (make-package "BALISE::OBJARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjArray.msg.html


(defclass balise::ObjArray
  :super ros::object
  :slots (_elements ))

(defmethod balise::ObjArray
  (:init
   (&key
    ((:elements __elements) ())
    )
   (send-super :init)
   (setq _elements __elements)
   self)
  (:elements
   (&rest __elements)
   (if (keywordp (car __elements))
       (send* _elements __elements)
     (progn
       (if __elements (setq _elements (car __elements)))
       _elements)))
  (:serialization-length
   ()
   (+
    ;; balise/Obj[] _elements
    (apply #'+ (send-all _elements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; balise/Obj[] _elements
     (write-long (length _elements) s)
     (dolist (elem _elements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; balise/Obj[] _elements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _elements (let (r) (dotimes (i n) (push (instance balise::Obj :init) r)) r))
     (dolist (elem- _elements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get balise::ObjArray :md5sum-) "bd1a346423c1712118b76ab7523541bc")
(setf (get balise::ObjArray :datatype-) "balise/ObjArray")
(setf (get balise::ObjArray :definition-)
      "Obj[] elements
================================================================================
MSG: balise/Obj
string dscript 
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



(provide :balise/ObjArray "bd1a346423c1712118b76ab7523541bc")


