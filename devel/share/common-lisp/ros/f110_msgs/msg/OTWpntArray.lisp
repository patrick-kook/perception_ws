; Auto-generated. Do not edit!


(cl:in-package f110_msgs-msg)


;//! \htmlinclude OTWpntArray.msg.html

(cl:defclass <OTWpntArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (last_switch_time
    :reader last_switch_time
    :initarg :last_switch_time
    :type cl:real
    :initform 0)
   (side_switch
    :reader side_switch
    :initarg :side_switch
    :type cl:boolean
    :initform cl:nil)
   (ot_side
    :reader ot_side
    :initarg :ot_side
    :type cl:string
    :initform "")
   (ot_line
    :reader ot_line
    :initarg :ot_line
    :type cl:string
    :initform "")
   (wpnts
    :reader wpnts
    :initarg :wpnts
    :type (cl:vector f110_msgs-msg:Wpnt)
   :initform (cl:make-array 0 :element-type 'f110_msgs-msg:Wpnt :initial-element (cl:make-instance 'f110_msgs-msg:Wpnt))))
)

(cl:defclass OTWpntArray (<OTWpntArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OTWpntArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OTWpntArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name f110_msgs-msg:<OTWpntArray> is deprecated: use f110_msgs-msg:OTWpntArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <OTWpntArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:header-val is deprecated.  Use f110_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'last_switch_time-val :lambda-list '(m))
(cl:defmethod last_switch_time-val ((m <OTWpntArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:last_switch_time-val is deprecated.  Use f110_msgs-msg:last_switch_time instead.")
  (last_switch_time m))

(cl:ensure-generic-function 'side_switch-val :lambda-list '(m))
(cl:defmethod side_switch-val ((m <OTWpntArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:side_switch-val is deprecated.  Use f110_msgs-msg:side_switch instead.")
  (side_switch m))

(cl:ensure-generic-function 'ot_side-val :lambda-list '(m))
(cl:defmethod ot_side-val ((m <OTWpntArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:ot_side-val is deprecated.  Use f110_msgs-msg:ot_side instead.")
  (ot_side m))

(cl:ensure-generic-function 'ot_line-val :lambda-list '(m))
(cl:defmethod ot_line-val ((m <OTWpntArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:ot_line-val is deprecated.  Use f110_msgs-msg:ot_line instead.")
  (ot_line m))

(cl:ensure-generic-function 'wpnts-val :lambda-list '(m))
(cl:defmethod wpnts-val ((m <OTWpntArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_msgs-msg:wpnts-val is deprecated.  Use f110_msgs-msg:wpnts instead.")
  (wpnts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OTWpntArray>) ostream)
  "Serializes a message object of type '<OTWpntArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'last_switch_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'last_switch_time) (cl:floor (cl:slot-value msg 'last_switch_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'side_switch) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ot_side))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ot_side))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ot_line))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ot_line))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'wpnts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'wpnts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OTWpntArray>) istream)
  "Deserializes a message object of type '<OTWpntArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'last_switch_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:slot-value msg 'side_switch) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ot_side) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ot_side) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ot_line) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ot_line) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'wpnts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'wpnts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'f110_msgs-msg:Wpnt))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OTWpntArray>)))
  "Returns string type for a message object of type '<OTWpntArray>"
  "f110_msgs/OTWpntArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OTWpntArray)))
  "Returns string type for a message object of type 'OTWpntArray"
  "f110_msgs/OTWpntArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OTWpntArray>)))
  "Returns md5sum for a message object of type '<OTWpntArray>"
  "ca31dbec903934bb444714f693d1ec7f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OTWpntArray)))
  "Returns md5sum for a message object of type 'OTWpntArray"
  "ca31dbec903934bb444714f693d1ec7f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OTWpntArray>)))
  "Returns full string definition for message of type '<OTWpntArray>"
  (cl:format cl:nil "std_msgs/Header header~%time last_switch_time~%bool side_switch~%string ot_side~%string ot_line~%Wpnt[] wpnts~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: f110_msgs/Wpnt~%int32 id~%~%# frenet coordinates~%float64 s_m~%float64 d_m~%~%# map coordinates~%float64 x_m~%float64 y_m~%~%# track bound distance~%float64 d_right~%float64 d_left~%~%# track information~%float64 psi_rad~%float64 kappa_radpm~%float64 vx_mps~%float64 ax_mps2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OTWpntArray)))
  "Returns full string definition for message of type 'OTWpntArray"
  (cl:format cl:nil "std_msgs/Header header~%time last_switch_time~%bool side_switch~%string ot_side~%string ot_line~%Wpnt[] wpnts~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: f110_msgs/Wpnt~%int32 id~%~%# frenet coordinates~%float64 s_m~%float64 d_m~%~%# map coordinates~%float64 x_m~%float64 y_m~%~%# track bound distance~%float64 d_right~%float64 d_left~%~%# track information~%float64 psi_rad~%float64 kappa_radpm~%float64 vx_mps~%float64 ax_mps2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OTWpntArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     1
     4 (cl:length (cl:slot-value msg 'ot_side))
     4 (cl:length (cl:slot-value msg 'ot_line))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'wpnts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OTWpntArray>))
  "Converts a ROS message object to a list"
  (cl:list 'OTWpntArray
    (cl:cons ':header (header msg))
    (cl:cons ':last_switch_time (last_switch_time msg))
    (cl:cons ':side_switch (side_switch msg))
    (cl:cons ':ot_side (ot_side msg))
    (cl:cons ':ot_line (ot_line msg))
    (cl:cons ':wpnts (wpnts msg))
))
