; Auto-generated. Do not edit!


(cl:in-package balise_msgs-msg)


;//! \htmlinclude PositionGameElements.msg.html

(cl:defclass <PositionGameElements> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform ""))
)

(cl:defclass PositionGameElements (<PositionGameElements>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionGameElements>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionGameElements)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name balise_msgs-msg:<PositionGameElements> is deprecated: use balise_msgs-msg:PositionGameElements instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <PositionGameElements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise_msgs-msg:x-val is deprecated.  Use balise_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <PositionGameElements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise_msgs-msg:y-val is deprecated.  Use balise_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <PositionGameElements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise_msgs-msg:theta-val is deprecated.  Use balise_msgs-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <PositionGameElements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise_msgs-msg:type-val is deprecated.  Use balise_msgs-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionGameElements>) ostream)
  "Serializes a message object of type '<PositionGameElements>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'theta)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionGameElements>) istream)
  "Deserializes a message object of type '<PositionGameElements>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'theta) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionGameElements>)))
  "Returns string type for a message object of type '<PositionGameElements>"
  "balise_msgs/PositionGameElements")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionGameElements)))
  "Returns string type for a message object of type 'PositionGameElements"
  "balise_msgs/PositionGameElements")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionGameElements>)))
  "Returns md5sum for a message object of type '<PositionGameElements>"
  "18f807f0bff0003047e6e888eca3bfa5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionGameElements)))
  "Returns md5sum for a message object of type 'PositionGameElements"
  "18f807f0bff0003047e6e888eca3bfa5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionGameElements>)))
  "Returns full string definition for message of type '<PositionGameElements>"
  (cl:format cl:nil "int32 x~%int32 y ~%int32 theta~%string type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionGameElements)))
  "Returns full string definition for message of type 'PositionGameElements"
  (cl:format cl:nil "int32 x~%int32 y ~%int32 theta~%string type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionGameElements>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionGameElements>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionGameElements
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':type (type msg))
))
