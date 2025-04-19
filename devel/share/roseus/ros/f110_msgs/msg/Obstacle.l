;; Auto-generated. Do not edit!


(when (boundp 'f110_msgs::Obstacle)
  (if (not (find-package "F110_MSGS"))
    (make-package "F110_MSGS"))
  (shadow 'Obstacle (find-package "F110_MSGS")))
(unless (find-package "F110_MSGS::OBSTACLE")
  (make-package "F110_MSGS::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html


(defclass f110_msgs::Obstacle
  :super ros::object
  :slots (_id _s_start _s_end _d_right _d_left _is_actually_a_gap _s_center _d_center _size _vs _vd _s_var _d_var _vs_var _vd_var _is_static _is_visible ))

(defmethod f110_msgs::Obstacle
  (:init
   (&key
    ((:id __id) 0)
    ((:s_start __s_start) 0.0)
    ((:s_end __s_end) 0.0)
    ((:d_right __d_right) 0.0)
    ((:d_left __d_left) 0.0)
    ((:is_actually_a_gap __is_actually_a_gap) nil)
    ((:s_center __s_center) 0.0)
    ((:d_center __d_center) 0.0)
    ((:size __size) 0.0)
    ((:vs __vs) 0.0)
    ((:vd __vd) 0.0)
    ((:s_var __s_var) 0.0)
    ((:d_var __d_var) 0.0)
    ((:vs_var __vs_var) 0.0)
    ((:vd_var __vd_var) 0.0)
    ((:is_static __is_static) nil)
    ((:is_visible __is_visible) nil)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _s_start (float __s_start))
   (setq _s_end (float __s_end))
   (setq _d_right (float __d_right))
   (setq _d_left (float __d_left))
   (setq _is_actually_a_gap __is_actually_a_gap)
   (setq _s_center (float __s_center))
   (setq _d_center (float __d_center))
   (setq _size (float __size))
   (setq _vs (float __vs))
   (setq _vd (float __vd))
   (setq _s_var (float __s_var))
   (setq _d_var (float __d_var))
   (setq _vs_var (float __vs_var))
   (setq _vd_var (float __vd_var))
   (setq _is_static __is_static)
   (setq _is_visible __is_visible)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:s_start
   (&optional __s_start)
   (if __s_start (setq _s_start __s_start)) _s_start)
  (:s_end
   (&optional __s_end)
   (if __s_end (setq _s_end __s_end)) _s_end)
  (:d_right
   (&optional __d_right)
   (if __d_right (setq _d_right __d_right)) _d_right)
  (:d_left
   (&optional __d_left)
   (if __d_left (setq _d_left __d_left)) _d_left)
  (:is_actually_a_gap
   (&optional (__is_actually_a_gap :null))
   (if (not (eq __is_actually_a_gap :null)) (setq _is_actually_a_gap __is_actually_a_gap)) _is_actually_a_gap)
  (:s_center
   (&optional __s_center)
   (if __s_center (setq _s_center __s_center)) _s_center)
  (:d_center
   (&optional __d_center)
   (if __d_center (setq _d_center __d_center)) _d_center)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:vs
   (&optional __vs)
   (if __vs (setq _vs __vs)) _vs)
  (:vd
   (&optional __vd)
   (if __vd (setq _vd __vd)) _vd)
  (:s_var
   (&optional __s_var)
   (if __s_var (setq _s_var __s_var)) _s_var)
  (:d_var
   (&optional __d_var)
   (if __d_var (setq _d_var __d_var)) _d_var)
  (:vs_var
   (&optional __vs_var)
   (if __vs_var (setq _vs_var __vs_var)) _vs_var)
  (:vd_var
   (&optional __vd_var)
   (if __vd_var (setq _vd_var __vd_var)) _vd_var)
  (:is_static
   (&optional (__is_static :null))
   (if (not (eq __is_static :null)) (setq _is_static __is_static)) _is_static)
  (:is_visible
   (&optional (__is_visible :null))
   (if (not (eq __is_visible :null)) (setq _is_visible __is_visible)) _is_visible)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float64 _s_start
    8
    ;; float64 _s_end
    8
    ;; float64 _d_right
    8
    ;; float64 _d_left
    8
    ;; bool _is_actually_a_gap
    1
    ;; float64 _s_center
    8
    ;; float64 _d_center
    8
    ;; float64 _size
    8
    ;; float64 _vs
    8
    ;; float64 _vd
    8
    ;; float64 _s_var
    8
    ;; float64 _d_var
    8
    ;; float64 _vs_var
    8
    ;; float64 _vd_var
    8
    ;; bool _is_static
    1
    ;; bool _is_visible
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; float64 _s_start
       (sys::poke _s_start (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _s_end
       (sys::poke _s_end (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_right
       (sys::poke _d_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_left
       (sys::poke _d_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _is_actually_a_gap
       (if _is_actually_a_gap (write-byte -1 s) (write-byte 0 s))
     ;; float64 _s_center
       (sys::poke _s_center (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_center
       (sys::poke _d_center (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _size
       (sys::poke _size (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vs
       (sys::poke _vs (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vd
       (sys::poke _vd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _s_var
       (sys::poke _s_var (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_var
       (sys::poke _d_var (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vs_var
       (sys::poke _vs_var (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vd_var
       (sys::poke _vd_var (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _is_static
       (if _is_static (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_visible
       (if _is_visible (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _s_start
     (setq _s_start (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _s_end
     (setq _s_end (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_right
     (setq _d_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_left
     (setq _d_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _is_actually_a_gap
     (setq _is_actually_a_gap (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _s_center
     (setq _s_center (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_center
     (setq _d_center (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _size
     (setq _size (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vs
     (setq _vs (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vd
     (setq _vd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _s_var
     (setq _s_var (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_var
     (setq _d_var (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vs_var
     (setq _vs_var (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vd_var
     (setq _vd_var (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _is_static
     (setq _is_static (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_visible
     (setq _is_visible (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get f110_msgs::Obstacle :md5sum-) "778e7375210689bb1da0fcff69e149b8")
(setf (get f110_msgs::Obstacle :datatype-) "f110_msgs/Obstacle")
(setf (get f110_msgs::Obstacle :definition-)
      "int32 id

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



(provide :f110_msgs/Obstacle "778e7375210689bb1da0fcff69e149b8")


