; Auto-generated. Do not edit!


(cl:in-package chatroom-msg)


;//! \htmlinclude ChatMessage.msg.html

(cl:defclass <ChatMessage> (roslisp-msg-protocol:ros-message)
  ((sender
    :reader sender
    :initarg :sender
    :type cl:string
    :initform "")
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ChatMessage (<ChatMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChatMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChatMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chatroom-msg:<ChatMessage> is deprecated: use chatroom-msg:ChatMessage instead.")))

(cl:ensure-generic-function 'sender-val :lambda-list '(m))
(cl:defmethod sender-val ((m <ChatMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chatroom-msg:sender-val is deprecated.  Use chatroom-msg:sender instead.")
  (sender m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ChatMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chatroom-msg:message-val is deprecated.  Use chatroom-msg:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChatMessage>) ostream)
  "Serializes a message object of type '<ChatMessage>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sender))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sender))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChatMessage>) istream)
  "Deserializes a message object of type '<ChatMessage>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sender) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sender) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChatMessage>)))
  "Returns string type for a message object of type '<ChatMessage>"
  "chatroom/ChatMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChatMessage)))
  "Returns string type for a message object of type 'ChatMessage"
  "chatroom/ChatMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChatMessage>)))
  "Returns md5sum for a message object of type '<ChatMessage>"
  "985ef42f02a992c58d5de728edeb121e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChatMessage)))
  "Returns md5sum for a message object of type 'ChatMessage"
  "985ef42f02a992c58d5de728edeb121e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChatMessage>)))
  "Returns full string definition for message of type '<ChatMessage>"
  (cl:format cl:nil "string sender~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChatMessage)))
  "Returns full string definition for message of type 'ChatMessage"
  (cl:format cl:nil "string sender~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChatMessage>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'sender))
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChatMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'ChatMessage
    (cl:cons ':sender (sender msg))
    (cl:cons ':message (message msg))
))
