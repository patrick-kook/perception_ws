;; Auto-generated. Do not edit!


(when (boundp 'f110_msgs::OTWpntArray)
  (if (not (find-package "F110_MSGS"))
    (make-package "F110_MSGS"))
  (shadow 'OTWpntArray (find-package "F110_MSGS")))
(unless (find-package "F110_MSGS::OTWPNTARRAY")
  (make-package "F110_MSGS::OTWPNTARRAY"))

(in-package "ROS")
;;//! \htmlinclude OTWpntArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass f110_msgs::OTWpntArray
  :super ros::object
  :slots (_header _last_switch_time _side_switch _ot_side _ot_line _wpnts ))

(defmethod f110_msgs::OTWpntArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:last_switch_time __last_switch_time) (instance ros::time :init))
    ((:side_switch __side_switch) nil)
    ((:ot_side __ot_side) "")
    ((:ot_line __ot_line) "")
    ((:wpnts __wpnts) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _last_switch_time __last_switch_time)
   (setq _side_switch __side_switch)
   (setq _ot_side (string __ot_side))
   (setq _ot_line (string __ot_line))
   (setq _wpnts __wpnts)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:last_switch_time
   (&optional __last_switch_time)
   (if __last_switch_time (setq _last_switch_time __last_switch_time)) _last_switch_time)
  (:side_switch
   (&optional (__side_switch :null))
   (if (not (eq __side_switch :null)) (setq _side_switch __side_switch)) _side_switch)
  (:ot_side
   (&optional __ot_side)
   (if __ot_side (setq _ot_side __ot_side)) _ot_side)
  (:ot_line
   (&optional __ot_line)
   (if __ot_line (setq _ot_line __ot_line)) _ot_line)
  (:wpnts
   (&rest __wpnts)
   (if (keywordp (car __wpnts))
       (send* _wpnts __wpnts)
     (progn
       (if __wpnts (setq _wpnts (car __wpnts)))
       _wpnts)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _last_switch_time
    8
    ;; bool _side_switch
    1
    ;; string _ot_side
    4 (length _ot_side)
    ;; string _ot_line
    4 (length _ot_line)
    ;; f110_msgs/Wpnt[] _wpnts
    (apply #'+ (send-all _wpnts :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _last_switch_time
       (write-long (send _last_switch_time :sec) s) (write-long (send _last_switch_time :nsec) s)
     ;; bool _side_switch
       (if _side_switch (write-byte -1 s) (write-byte 0 s))
     ;; string _ot_side
       (write-long (length _ot_side) s) (princ _ot_side s)
     ;; string _ot_line
       (write-long (length _ot_line) s) (princ _ot_line s)
     ;; f110_msgs/Wpnt[] _wpnts
     (write-long (length _wpnts) s)
     (dolist (elem _wpnts)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _last_switch_time
     (send _last_switch_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _last_switch_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _side_switch
     (setq _side_switch (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _ot_side
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ot_side (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _ot_line
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ot_line (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; f110_msgs/Wpnt[] _wpnts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wpnts (let (r) (dotimes (i n) (push (instance f110_msgs::Wpnt :init) r)) r))
     (dolist (elem- _wpnts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get f110_msgs::OTWpntArray :md5sum-) "ca31dbec903934bb444714f693d1ec7f")
(setf (get f110_msgs::OTWpntArray :datatype-) "f110_msgs/OTWpntArray")
(setf (get f110_msgs::OTWpntArray :definition-)
      "std_msgs/Header header
time last_switch_time
bool side_switch
string ot_side
string ot_line
Wpnt[] wpnts


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: f110_msgs/Wpnt
int32 id

# frenet coordinates
float64 s_m
float64 d_m

# map coordinates
float64 x_m
float64 y_m

# track bound distance
float64 d_right
float64 d_left

# track information
float64 psi_rad
float64 kappa_radpm
float64 vx_mps
float64 ax_mps2


")



(provide :f110_msgs/OTWpntArray "ca31dbec903934bb444714f693d1ec7f")


