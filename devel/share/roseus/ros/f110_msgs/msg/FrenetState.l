;; Auto-generated. Do not edit!


(when (boundp 'f110_msgs::FrenetState)
  (if (not (find-package "F110_MSGS"))
    (make-package "F110_MSGS"))
  (shadow 'FrenetState (find-package "F110_MSGS")))
(unless (find-package "F110_MSGS::FRENETSTATE")
  (make-package "F110_MSGS::FRENETSTATE"))

(in-package "ROS")
;;//! \htmlinclude FrenetState.msg.html


(defclass f110_msgs::FrenetState
  :super ros::object
  :slots (_s _d _v ))

(defmethod f110_msgs::FrenetState
  (:init
   (&key
    ((:s __s) 0.0)
    ((:d __d) 0.0)
    ((:v __v) 0.0)
    )
   (send-super :init)
   (setq _s (float __s))
   (setq _d (float __d))
   (setq _v (float __v))
   self)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:serialization-length
   ()
   (+
    ;; float64 _s
    8
    ;; float64 _d
    8
    ;; float64 _v
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _s
       (sys::poke _s (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d
       (sys::poke _d (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _s
     (setq _s (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d
     (setq _d (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get f110_msgs::FrenetState :md5sum-) "0f35f65d8c6f765f27afccad48d013c7")
(setf (get f110_msgs::FrenetState :datatype-) "f110_msgs/FrenetState")
(setf (get f110_msgs::FrenetState :definition-)
      "float64 s       # The longitudinal position along the Frenet path
float64 d       # The lateral distance from the Frenet path
float64 v       # The longitudinal velocity (m/s)


")



(provide :f110_msgs/FrenetState "0f35f65d8c6f765f27afccad48d013c7")


