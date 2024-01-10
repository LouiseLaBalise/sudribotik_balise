; Auto-generated. Do not edit!


(cl:in-package balise-msg)


;//! \htmlinclude Obj.msg.html

(cl:defclass <Obj> (roslisp-msg-protocol:ros-message)
  ((dscript
    :reader dscript
    :initarg :dscript
    :type cl:string
    :initform "")
   (ident
    :reader ident
    :initarg :ident
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass Obj (<Obj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Obj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Obj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name balise-msg:<Obj> is deprecated: use balise-msg:Obj instead.")))

(cl:ensure-generic-function 'dscript-val :lambda-list '(m))
(cl:defmethod dscript-val ((m <Obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:dscript-val is deprecated.  Use balise-msg:dscript instead.")
  (dscript m))

(cl:ensure-generic-function 'ident-val :lambda-list '(m))
(cl:defmethod ident-val ((m <Obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:ident-val is deprecated.  Use balise-msg:ident instead.")
  (ident m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:position-val is deprecated.  Use balise-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <Obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:theta-val is deprecated.  Use balise-msg:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Obj>) ostream)
  "Serializes a message object of type '<Obj>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dscript))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dscript))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ident)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ident)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ident)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ident)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Obj>) istream)
  "Deserializes a message object of type '<Obj>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dscript) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dscript) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ident)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ident)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ident)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ident)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Obj>)))
  "Returns string type for a message object of type '<Obj>"
  "balise/Obj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Obj)))
  "Returns string type for a message object of type 'Obj"
  "balise/Obj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Obj>)))
  "Returns md5sum for a message object of type '<Obj>"
  "f95b3051f8929661f923cb09696172fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Obj)))
  "Returns md5sum for a message object of type 'Obj"
  "f95b3051f8929661f923cb09696172fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Obj>)))
  "Returns full string definition for message of type '<Obj>"
  (cl:format cl:nil "string dscript ~%uint32 ident~%geometry_msgs/Vector3 position~%float64 theta~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Obj)))
  "Returns full string definition for message of type 'Obj"
  (cl:format cl:nil "string dscript ~%uint32 ident~%geometry_msgs/Vector3 position~%float64 theta~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Obj>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'dscript))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Obj>))
  "Converts a ROS message object to a list"
  (cl:list 'Obj
    (cl:cons ':dscript (dscript msg))
    (cl:cons ':ident (ident msg))
    (cl:cons ':position (position msg))
    (cl:cons ':theta (theta msg))
))
