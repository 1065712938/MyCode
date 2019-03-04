; Auto-generated. Do not edit!


(cl:in-package custom_msg_topic-msg)


;//! \htmlinclude custom_msg.msg.html

(cl:defclass <custom_msg> (roslisp-msg-protocol:ros-message)
  ((levels_of_anger
    :reader levels_of_anger
    :initarg :levels_of_anger
    :type cl:float
    :initform 0.0)
   (min_lidar_data
    :reader min_lidar_data
    :initarg :min_lidar_data
    :type cl:float
    :initform 0.0)
   (Avoidance_Classification_group
    :reader Avoidance_Classification_group
    :initarg :Avoidance_Classification_group
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (Speed_change_of_obstacle
    :reader Speed_change_of_obstacle
    :initarg :Speed_change_of_obstacle
    :type cl:float
    :initform 0.0))
)

(cl:defclass custom_msg (<custom_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <custom_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'custom_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg_topic-msg:<custom_msg> is deprecated: use custom_msg_topic-msg:custom_msg instead.")))

(cl:ensure-generic-function 'levels_of_anger-val :lambda-list '(m))
(cl:defmethod levels_of_anger-val ((m <custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg_topic-msg:levels_of_anger-val is deprecated.  Use custom_msg_topic-msg:levels_of_anger instead.")
  (levels_of_anger m))

(cl:ensure-generic-function 'min_lidar_data-val :lambda-list '(m))
(cl:defmethod min_lidar_data-val ((m <custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg_topic-msg:min_lidar_data-val is deprecated.  Use custom_msg_topic-msg:min_lidar_data instead.")
  (min_lidar_data m))

(cl:ensure-generic-function 'Avoidance_Classification_group-val :lambda-list '(m))
(cl:defmethod Avoidance_Classification_group-val ((m <custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg_topic-msg:Avoidance_Classification_group-val is deprecated.  Use custom_msg_topic-msg:Avoidance_Classification_group instead.")
  (Avoidance_Classification_group m))

(cl:ensure-generic-function 'Speed_change_of_obstacle-val :lambda-list '(m))
(cl:defmethod Speed_change_of_obstacle-val ((m <custom_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg_topic-msg:Speed_change_of_obstacle-val is deprecated.  Use custom_msg_topic-msg:Speed_change_of_obstacle instead.")
  (Speed_change_of_obstacle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <custom_msg>) ostream)
  "Serializes a message object of type '<custom_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'levels_of_anger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_lidar_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Avoidance_Classification_group))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Avoidance_Classification_group))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Speed_change_of_obstacle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <custom_msg>) istream)
  "Deserializes a message object of type '<custom_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'levels_of_anger) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_lidar_data) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Avoidance_Classification_group) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Avoidance_Classification_group)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Speed_change_of_obstacle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<custom_msg>)))
  "Returns string type for a message object of type '<custom_msg>"
  "custom_msg_topic/custom_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'custom_msg)))
  "Returns string type for a message object of type 'custom_msg"
  "custom_msg_topic/custom_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<custom_msg>)))
  "Returns md5sum for a message object of type '<custom_msg>"
  "56c5740684da4cec8bf62501dd7b9504")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'custom_msg)))
  "Returns md5sum for a message object of type 'custom_msg"
  "56c5740684da4cec8bf62501dd7b9504")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<custom_msg>)))
  "Returns full string definition for message of type '<custom_msg>"
  (cl:format cl:nil "float32 levels_of_anger~%float32 min_lidar_data~%float32[] Avoidance_Classification_group~%float32 Speed_change_of_obstacle ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'custom_msg)))
  "Returns full string definition for message of type 'custom_msg"
  (cl:format cl:nil "float32 levels_of_anger~%float32 min_lidar_data~%float32[] Avoidance_Classification_group~%float32 Speed_change_of_obstacle ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <custom_msg>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Avoidance_Classification_group) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <custom_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'custom_msg
    (cl:cons ':levels_of_anger (levels_of_anger msg))
    (cl:cons ':min_lidar_data (min_lidar_data msg))
    (cl:cons ':Avoidance_Classification_group (Avoidance_Classification_group msg))
    (cl:cons ':Speed_change_of_obstacle (Speed_change_of_obstacle msg))
))
