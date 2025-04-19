; Auto-generated. Do not edit!


(cl:in-package f110_msgs-msg)


;//! \htmlinclude ObstacleArray.msg.html

(cl:defclass <ObstacleArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector f110_msgs-msg:Obstacle)
   :initform (cl:make-array 0 :element-type 'f110_msgs-msg:Obstacle :initial-element (cl:make-instance 'f110_msgs-msg:Obstacle))))
)

(cl:defclass ObstacleArray (<ObstacleArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name f110_msgs-msg:<ObstacleArray> is deprecated: use f110_msgs-msg:ObstacleArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:header-val is deprecated.  Use f110_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <ObstacleArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:obstacles-val is deprecated.  Use f110_msgs-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleArray>) ostream)
  "Serializes a message object of type '<ObstacleArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleArray>) istream)
  "Deserializes a message object of type '<ObstacleArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'f110_msgs-msg:Obstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleArray>)))
  "Returns string type for a message object of type '<ObstacleArray>"
  "f110_msgs/ObstacleArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleArray)))
  "Returns string type for a message object of type 'ObstacleArray"
  "f110_msgs/ObstacleArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleArray>)))
  "Returns md5sum for a message object of type '<ObstacleArray>"
  "eaec48ede41b193c9dbb014a089c085d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleArray)))
  "Returns md5sum for a message object of type 'ObstacleArray"
  "eaec48ede41b193c9dbb014a089c085d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleArray>)))
  "Returns full string definition for message of type '<ObstacleArray>"
  (cl:format cl:nil "std_msgs/Header header~%Obstacle[] obstacles~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: f110_msgs/Obstacle~%int32 id~%~%float64 s_start~%float64 s_end~%float64 d_right           # defined as right bound of the obstacle~%float64 d_left            # defined as left bound of the obstacle~%bool is_actually_a_gap    # used by the frenet planner to choos points through which we want to go~%float64 s_center~%float64 d_center~%float64 size~%float64 vs~%float64 vd~%float64 s_var~%float64 d_var~%float64 vs_var~%float64 vd_var~%bool is_static~%bool is_visible~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleArray)))
  "Returns full string definition for message of type 'ObstacleArray"
  (cl:format cl:nil "std_msgs/Header header~%Obstacle[] obstacles~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: f110_msgs/Obstacle~%int32 id~%~%float64 s_start~%float64 s_end~%float64 d_right           # defined as right bound of the obstacle~%float64 d_left            # defined as left bound of the obstacle~%bool is_actually_a_gap    # used by the frenet planner to choos points through which we want to go~%float64 s_center~%float64 d_center~%float64 size~%float64 vs~%float64 vd~%float64 s_var~%float64 d_var~%float64 vs_var~%float64 vd_var~%bool is_static~%bool is_visible~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleArray
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
))
