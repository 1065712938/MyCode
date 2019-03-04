;; Auto-generated. Do not edit!


(when (boundp 'custom_msg_topic::custom_msg)
  (if (not (find-package "CUSTOM_MSG_TOPIC"))
    (make-package "CUSTOM_MSG_TOPIC"))
  (shadow 'custom_msg (find-package "CUSTOM_MSG_TOPIC")))
(unless (find-package "CUSTOM_MSG_TOPIC::CUSTOM_MSG")
  (make-package "CUSTOM_MSG_TOPIC::CUSTOM_MSG"))

(in-package "ROS")
;;//! \htmlinclude custom_msg.msg.html


(defclass custom_msg_topic::custom_msg
  :super ros::object
  :slots (_levels_of_anger _min_lidar_data _Avoidance_Classification_group _Speed_change_of_obstacle ))

(defmethod custom_msg_topic::custom_msg
  (:init
   (&key
    ((:levels_of_anger __levels_of_anger) 0.0)
    ((:min_lidar_data __min_lidar_data) 0.0)
    ((:Avoidance_Classification_group __Avoidance_Classification_group) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:Speed_change_of_obstacle __Speed_change_of_obstacle) 0.0)
    )
   (send-super :init)
   (setq _levels_of_anger (float __levels_of_anger))
   (setq _min_lidar_data (float __min_lidar_data))
   (setq _Avoidance_Classification_group __Avoidance_Classification_group)
   (setq _Speed_change_of_obstacle (float __Speed_change_of_obstacle))
   self)
  (:levels_of_anger
   (&optional __levels_of_anger)
   (if __levels_of_anger (setq _levels_of_anger __levels_of_anger)) _levels_of_anger)
  (:min_lidar_data
   (&optional __min_lidar_data)
   (if __min_lidar_data (setq _min_lidar_data __min_lidar_data)) _min_lidar_data)
  (:Avoidance_Classification_group
   (&optional __Avoidance_Classification_group)
   (if __Avoidance_Classification_group (setq _Avoidance_Classification_group __Avoidance_Classification_group)) _Avoidance_Classification_group)
  (:Speed_change_of_obstacle
   (&optional __Speed_change_of_obstacle)
   (if __Speed_change_of_obstacle (setq _Speed_change_of_obstacle __Speed_change_of_obstacle)) _Speed_change_of_obstacle)
  (:serialization-length
   ()
   (+
    ;; float32 _levels_of_anger
    4
    ;; float32 _min_lidar_data
    4
    ;; float32[] _Avoidance_Classification_group
    (* 4    (length _Avoidance_Classification_group)) 4
    ;; float32 _Speed_change_of_obstacle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _levels_of_anger
       (sys::poke _levels_of_anger (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_lidar_data
       (sys::poke _min_lidar_data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _Avoidance_Classification_group
     (write-long (length _Avoidance_Classification_group) s)
     (dotimes (i (length _Avoidance_Classification_group))
       (sys::poke (elt _Avoidance_Classification_group i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _Speed_change_of_obstacle
       (sys::poke _Speed_change_of_obstacle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _levels_of_anger
     (setq _levels_of_anger (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_lidar_data
     (setq _min_lidar_data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _Avoidance_Classification_group
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Avoidance_Classification_group (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _Avoidance_Classification_group i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _Speed_change_of_obstacle
     (setq _Speed_change_of_obstacle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msg_topic::custom_msg :md5sum-) "56c5740684da4cec8bf62501dd7b9504")
(setf (get custom_msg_topic::custom_msg :datatype-) "custom_msg_topic/custom_msg")
(setf (get custom_msg_topic::custom_msg :definition-)
      "float32 levels_of_anger
float32 min_lidar_data
float32[] Avoidance_Classification_group
float32 Speed_change_of_obstacle 

")



(provide :custom_msg_topic/custom_msg "56c5740684da4cec8bf62501dd7b9504")


