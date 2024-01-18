;; Auto-generated. Do not edit!


(when (boundp 'balise_msgs::Score)
  (if (not (find-package "BALISE_MSGS"))
    (make-package "BALISE_MSGS"))
  (shadow 'Score (find-package "BALISE_MSGS")))
(unless (find-package "BALISE_MSGS::SCORE")
  (make-package "BALISE_MSGS::SCORE"))

(in-package "ROS")
;;//! \htmlinclude Score.msg.html


(defclass balise_msgs::Score
  :super ros::object
  :slots (_score ))

(defmethod balise_msgs::Score
  (:init
   (&key
    ((:score __score) 0)
    )
   (send-super :init)
   (setq _score (round __score))
   self)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:serialization-length
   ()
   (+
    ;; int32 _score
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _score
       (write-long _score s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _score
     (setq _score (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get balise_msgs::Score :md5sum-) "2ed963831938da1fc8151b77dec7741f")
(setf (get balise_msgs::Score :datatype-) "balise_msgs/Score")
(setf (get balise_msgs::Score :definition-)
      "int32 score
")



(provide :balise_msgs/Score "2ed963831938da1fc8151b77dec7741f")


