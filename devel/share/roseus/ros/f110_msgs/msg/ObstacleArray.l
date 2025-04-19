;; Auto-generated. Do not edit!


(when (boundp 'f110_msgs::ObstacleArray)
  (if (not (find-package "F110_MSGS"))
    (make-package "F110_MSGS"))
  (shadow 'ObstacleArray (find-package "F110_MSGS")))
(unless (find-package "F110_MSGS::OBSTACLEARRAY")
  (make-package "F110_MSGS::OBSTACLEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObstacleArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass f110_msgs::ObstacleArray
  :super ros::object
  :slots (_header _obstacles ))

(defmethod f110_msgs::ObstacleArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; f110_msgs/Obstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; f110_msgs/Obstacle[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; f110_msgs/Obstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance f110_msgs::Obstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get f110_msgs::ObstacleArray :md5sum-) "eaec48ede41b193c9dbb014a089c085d")
(setf (get f110_msgs::ObstacleArray :datatype-) "f110_msgs/ObstacleArray")
(setf (get f110_msgs::ObstacleArray :definition-)
      "std_msgs/Header header
Obstacle[] obstacles
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
MSG: f110_msgs/Obstacle
int32 id

float64 s_start
float64 s_end
float64 d_right           # defined as right bound of the obstacle
float64 d_left            # defined as left bound of the obstacle
bool is_actually_a_gap    # used by the frenet planner to choos points through which we want to go
float64 s_center
float64 d_center
float64 size
float64 vs
float64 vd
float64 s_var
float64 d_var
float64 vs_var
float64 vd_var
bool is_static
bool is_visible
")



(provide :f110_msgs/ObstacleArray "eaec48ede41b193c9dbb014a089c085d")


