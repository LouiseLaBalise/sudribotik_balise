; Auto-generated. Do not edit!


(cl:in-package balise-msg)


;//! \htmlinclude ObjArray.msg.html

(cl:defclass <ObjArray> (roslisp-msg-protocol:ros-message)
  ((elements
    :reader elements
    :initarg :elements
    :type (cl:vector balise-msg:Obj)
   :initform (cl:make-array 0 :element-type 'balise-msg:Obj :initial-element (cl:make-instance 'balise-msg:Obj))))
)

(cl:defclass ObjArray (<ObjArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name balise-msg:<ObjArray> is deprecated: use balise-msg:ObjArray instead.")))

(cl:ensure-generic-function 'elements-val :lambda-list '(m))
(cl:defmethod elements-val ((m <ObjArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:elements-val is deprecated.  Use balise-msg:elements instead.")
  (elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjArray>) ostream)
  "Serializes a message object of type '<ObjArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjArray>) istream)
  "Deserializes a message object of type '<ObjArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'balise-msg:Obj))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjArray>)))
  "Returns string type for a message object of type '<ObjArray>"
  "balise/ObjArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjArray)))
  "Returns string type for a message object of type 'ObjArray"
  "balise/ObjArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjArray>)))
  "Returns md5sum for a message object of type '<ObjArray>"
  "bd1a346423c1712118b76ab7523541bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjArray)))
  "Returns md5sum for a message object of type 'ObjArray"
  "bd1a346423c1712118b76ab7523541bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjArray>)))
  "Returns full string definition for message of type '<ObjArray>"
  (cl:format cl:nil "Obj[] elements~%================================================================================~%MSG: balise/Obj~%string dscript ~%uint32 ident~%geometry_msgs/Vector3 position~%float64 theta~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjArray)))
  "Returns full string definition for message of type 'ObjArray"
  (cl:format cl:nil "Obj[] elements~%================================================================================~%MSG: balise/Obj~%string dscript ~%uint32 ident~%geometry_msgs/Vector3 position~%float64 theta~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjArray
    (cl:cons ':elements (elements msg))
))
