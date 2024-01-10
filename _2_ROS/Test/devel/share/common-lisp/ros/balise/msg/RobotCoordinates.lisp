; Auto-generated. Do not edit!


(cl:in-package balise-msg)


;//! \htmlinclude RobotCoordinates.msg.html

(cl:defclass <RobotCoordinates> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotCoordinates (<RobotCoordinates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotCoordinates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotCoordinates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name balise-msg:<RobotCoordinates> is deprecated: use balise-msg:RobotCoordinates instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <RobotCoordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:x-val is deprecated.  Use balise-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <RobotCoordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:y-val is deprecated.  Use balise-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <RobotCoordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:theta-val is deprecated.  Use balise-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <RobotCoordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise-msg:id-val is deprecated.  Use balise-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotCoordinates>) ostream)
  "Serializes a message object of type '<RobotCoordinates>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotCoordinates>) istream)
  "Deserializes a message object of type '<RobotCoordinates>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotCoordinates>)))
  "Returns string type for a message object of type '<RobotCoordinates>"
  "balise/RobotCoordinates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotCoordinates)))
  "Returns string type for a message object of type 'RobotCoordinates"
  "balise/RobotCoordinates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotCoordinates>)))
  "Returns md5sum for a message object of type '<RobotCoordinates>"
  "7315a950cf4918ab18c91703ecd24e3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotCoordinates)))
  "Returns md5sum for a message object of type 'RobotCoordinates"
  "7315a950cf4918ab18c91703ecd24e3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotCoordinates>)))
  "Returns full string definition for message of type '<RobotCoordinates>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%uint32 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotCoordinates)))
  "Returns full string definition for message of type 'RobotCoordinates"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%uint32 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotCoordinates>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotCoordinates>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotCoordinates
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':id (id msg))
))
