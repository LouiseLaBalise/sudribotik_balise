; Auto-generated. Do not edit!


(cl:in-package balise_msgs-msg)


;//! \htmlinclude ArrayPositionPx.msg.html

(cl:defclass <ArrayPositionPx> (roslisp-msg-protocol:ros-message)
  ((array_of_positionspx
    :reader array_of_positionspx
    :initarg :array_of_positionspx
    :type (cl:vector balise_msgs-msg:PositionPx)
   :initform (cl:make-array 0 :element-type 'balise_msgs-msg:PositionPx :initial-element (cl:make-instance 'balise_msgs-msg:PositionPx))))
)

(cl:defclass ArrayPositionPx (<ArrayPositionPx>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArrayPositionPx>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArrayPositionPx)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name balise_msgs-msg:<ArrayPositionPx> is deprecated: use balise_msgs-msg:ArrayPositionPx instead.")))

(cl:ensure-generic-function 'array_of_positionspx-val :lambda-list '(m))
(cl:defmethod array_of_positionspx-val ((m <ArrayPositionPx>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise_msgs-msg:array_of_positionspx-val is deprecated.  Use balise_msgs-msg:array_of_positionspx instead.")
  (array_of_positionspx m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArrayPositionPx>) ostream)
  "Serializes a message object of type '<ArrayPositionPx>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'array_of_positionspx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'array_of_positionspx))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArrayPositionPx>) istream)
  "Deserializes a message object of type '<ArrayPositionPx>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'array_of_positionspx) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'array_of_positionspx)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'balise_msgs-msg:PositionPx))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArrayPositionPx>)))
  "Returns string type for a message object of type '<ArrayPositionPx>"
  "balise_msgs/ArrayPositionPx")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArrayPositionPx)))
  "Returns string type for a message object of type 'ArrayPositionPx"
  "balise_msgs/ArrayPositionPx")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArrayPositionPx>)))
  "Returns md5sum for a message object of type '<ArrayPositionPx>"
  "4d42662441de878808f5bfe3b2bdeae4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArrayPositionPx)))
  "Returns md5sum for a message object of type 'ArrayPositionPx"
  "4d42662441de878808f5bfe3b2bdeae4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArrayPositionPx>)))
  "Returns full string definition for message of type '<ArrayPositionPx>"
  (cl:format cl:nil "PositionPx[] array_of_positionspx~%================================================================================~%MSG: balise_msgs/PositionPx~%int32 x~%int32 y ~%int32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArrayPositionPx)))
  "Returns full string definition for message of type 'ArrayPositionPx"
  (cl:format cl:nil "PositionPx[] array_of_positionspx~%================================================================================~%MSG: balise_msgs/PositionPx~%int32 x~%int32 y ~%int32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArrayPositionPx>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'array_of_positionspx) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArrayPositionPx>))
  "Converts a ROS message object to a list"
  (cl:list 'ArrayPositionPx
    (cl:cons ':array_of_positionspx (array_of_positionspx msg))
))
