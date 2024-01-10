;; Auto-generated. Do not edit!


(when (boundp 'balise::PlantCoordinates)
  (if (not (find-package "BALISE"))
    (make-package "BALISE"))
  (shadow 'PlantCoordinates (find-package "BALISE")))
(unless (find-package "BALISE::PLANTCOORDINATES")
  (make-package "BALISE::PLANTCOORDINATES"))

(in-package "ROS")
;;//! \htmlinclude PlantCoordinates.msg.html


(defclass balise::PlantCoordinates
  :super ros::object
  :slots (_x _y _theta _id ))

(defmethod balise::PlantCoordinates
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   (setq _id (round __id))
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
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _theta
    4
    ;; uint32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get balise::PlantCoordinates :md5sum-) "7315a950cf4918ab18c91703ecd24e3b")
(setf (get balise::PlantCoordinates :datatype-) "balise/PlantCoordinates")
(setf (get balise::PlantCoordinates :definition-)
      "float32 x
float32 y
float32 theta
uint32 id
")



(provide :balise/PlantCoordinates "7315a950cf4918ab18c91703ecd24e3b")


