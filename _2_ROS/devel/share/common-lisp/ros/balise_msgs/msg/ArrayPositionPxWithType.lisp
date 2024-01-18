; Auto-generated. Do not edit!


(cl:in-package balise_msgs-msg)


;//! \htmlinclude ArrayPositionPxWithType.msg.html

(cl:defclass <ArrayPositionPxWithType> (roslisp-msg-protocol:ros-message)
  ((array_of_positionspx_with_type
    :reader array_of_positionspx_with_type
    :initarg :array_of_positionspx_with_type
    :type (cl:vector balise_msgs-msg:PositionPxWithType)
   :initform (cl:make-array 0 :element-type 'balise_msgs-msg:PositionPxWithType :initial-element (cl:make-instance 'balise_msgs-msg:PositionPxWithType))))
)

(cl:defclass ArrayPositionPxWithType (<ArrayPositionPxWithType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArrayPositionPxWithType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArrayPositionPxWithType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name balise_msgs-msg:<ArrayPositionPxWithType> is deprecated: use balise_msgs-msg:ArrayPositionPxWithType instead.")))

(cl:ensure-generic-function 'array_of_positionspx_with_type-val :lambda-list '(m))
(cl:defmethod array_of_positionspx_with_type-val ((m <ArrayPositionPxWithType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise_msgs-msg:array_of_positionspx_with_type-val is deprecated.  Use balise_msgs-msg:array_of_positionspx_with_type instead.")
  (array_of_positionspx_with_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArrayPositionPxWithType>) ostream)
  "Serializes a message object of type '<ArrayPositionPxWithType>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'array_of_positionspx_with_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'array_of_positionspx_with_type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArrayPositionPxWithType>) istream)
  "Deserializes a message object of type '<ArrayPositionPxWithType>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'array_of_positionspx_with_type) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'array_of_positionspx_with_type)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'balise_msgs-msg:PositionPxWithType))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArrayPositionPxWithType>)))
  "Returns string type for a message object of type '<ArrayPositionPxWithType>"
  "balise_msgs/ArrayPositionPxWithType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArrayPositionPxWithType)))
  "Returns string type for a message object of type 'ArrayPositionPxWithType"
  "balise_msgs/ArrayPositionPxWithType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArrayPositionPxWithType>)))
  "Returns md5sum for a message object of type '<ArrayPositionPxWithType>"
  "bb0df7150251fed4c0b95568bc63c1f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArrayPositionPxWithType)))
  "Returns md5sum for a message object of type 'ArrayPositionPxWithType"
  "bb0df7150251fed4c0b95568bc63c1f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArrayPositionPxWithType>)))
  "Returns full string definition for message of type '<ArrayPositionPxWithType>"
  (cl:format cl:nil "PositionPxWithType[] array_of_positionspx_with_type~%================================================================================~%MSG: balise_msgs/PositionPxWithType~%int32 x~%int32 y ~%int32 theta~%string type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArrayPositionPxWithType)))
  "Returns full string definition for message of type 'ArrayPositionPxWithType"
  (cl:format cl:nil "PositionPxWithType[] array_of_positionspx_with_type~%================================================================================~%MSG: balise_msgs/PositionPxWithType~%int32 x~%int32 y ~%int32 theta~%string type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArrayPositionPxWithType>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'array_of_positionspx_with_type) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArrayPositionPxWithType>))
  "Converts a ROS message object to a list"
  (cl:list 'ArrayPositionPxWithType
    (cl:cons ':array_of_positionspx_with_type (array_of_positionspx_with_type msg))
))
