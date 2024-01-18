; Auto-generated. Do not edit!


(cl:in-package balise_msgs-msg)


;//! \htmlinclude Score.msg.html

(cl:defclass <Score> (roslisp-msg-protocol:ros-message)
  ((score
    :reader score
    :initarg :score
    :type cl:integer
    :initform 0))
)

(cl:defclass Score (<Score>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Score>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Score)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name balise_msgs-msg:<Score> is deprecated: use balise_msgs-msg:Score instead.")))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <Score>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader balise_msgs-msg:score-val is deprecated.  Use balise_msgs-msg:score instead.")
  (score m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Score>) ostream)
  "Serializes a message object of type '<Score>"
  (cl:let* ((signed (cl:slot-value msg 'score)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Score>) istream)
  "Deserializes a message object of type '<Score>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'score) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Score>)))
  "Returns string type for a message object of type '<Score>"
  "balise_msgs/Score")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Score)))
  "Returns string type for a message object of type 'Score"
  "balise_msgs/Score")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Score>)))
  "Returns md5sum for a message object of type '<Score>"
  "2ed963831938da1fc8151b77dec7741f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Score)))
  "Returns md5sum for a message object of type 'Score"
  "2ed963831938da1fc8151b77dec7741f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Score>)))
  "Returns full string definition for message of type '<Score>"
  (cl:format cl:nil "int32 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Score)))
  "Returns full string definition for message of type 'Score"
  (cl:format cl:nil "int32 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Score>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Score>))
  "Converts a ROS message object to a list"
  (cl:list 'Score
    (cl:cons ':score (score msg))
))
