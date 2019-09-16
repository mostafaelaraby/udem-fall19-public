;; Auto-generated. Do not edit!


(when (boundp 'ground_projection::GetGroundCoord)
  (if (not (find-package "GROUND_PROJECTION"))
    (make-package "GROUND_PROJECTION"))
  (shadow 'GetGroundCoord (find-package "GROUND_PROJECTION")))
(unless (find-package "GROUND_PROJECTION::GETGROUNDCOORD")
  (make-package "GROUND_PROJECTION::GETGROUNDCOORD"))
(unless (find-package "GROUND_PROJECTION::GETGROUNDCOORDREQUEST")
  (make-package "GROUND_PROJECTION::GETGROUNDCOORDREQUEST"))
(unless (find-package "GROUND_PROJECTION::GETGROUNDCOORDRESPONSE")
  (make-package "GROUND_PROJECTION::GETGROUNDCOORDRESPONSE"))

(in-package "ROS")

(if (not (find-package "DUCKIETOWN_MSGS"))
  (ros::roseus-add-msgs "duckietown_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ground_projection::GetGroundCoordRequest
  :super ros::object
  :slots (_normalized_uv ))

(defmethod ground_projection::GetGroundCoordRequest
  (:init
   (&key
    ((:normalized_uv __normalized_uv) (instance duckietown_msgs::Vector2D :init))
    )
   (send-super :init)
   (setq _normalized_uv __normalized_uv)
   self)
  (:normalized_uv
   (&rest __normalized_uv)
   (if (keywordp (car __normalized_uv))
       (send* _normalized_uv __normalized_uv)
     (progn
       (if __normalized_uv (setq _normalized_uv (car __normalized_uv)))
       _normalized_uv)))
  (:serialization-length
   ()
   (+
    ;; duckietown_msgs/Vector2D _normalized_uv
    (send _normalized_uv :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; duckietown_msgs/Vector2D _normalized_uv
       (send _normalized_uv :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; duckietown_msgs/Vector2D _normalized_uv
     (send _normalized_uv :deserialize buf ptr-) (incf ptr- (send _normalized_uv :serialization-length))
   ;;
   self)
  )

(defclass ground_projection::GetGroundCoordResponse
  :super ros::object
  :slots (_gp ))

(defmethod ground_projection::GetGroundCoordResponse
  (:init
   (&key
    ((:gp __gp) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _gp __gp)
   self)
  (:gp
   (&rest __gp)
   (if (keywordp (car __gp))
       (send* _gp __gp)
     (progn
       (if __gp (setq _gp (car __gp)))
       _gp)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _gp
    (send _gp :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _gp
       (send _gp :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _gp
     (send _gp :deserialize buf ptr-) (incf ptr- (send _gp :serialization-length))
   ;;
   self)
  )

(defclass ground_projection::GetGroundCoord
  :super ros::object
  :slots ())

(setf (get ground_projection::GetGroundCoord :md5sum-) "556869c22d708d07f00608b52f833756")
(setf (get ground_projection::GetGroundCoord :datatype-) "ground_projection/GetGroundCoord")
(setf (get ground_projection::GetGroundCoord :request) ground_projection::GetGroundCoordRequest)
(setf (get ground_projection::GetGroundCoord :response) ground_projection::GetGroundCoordResponse)

(defmethod ground_projection::GetGroundCoordRequest
  (:response () (instance ground_projection::GetGroundCoordResponse :init)))

(setf (get ground_projection::GetGroundCoordRequest :md5sum-) "556869c22d708d07f00608b52f833756")
(setf (get ground_projection::GetGroundCoordRequest :datatype-) "ground_projection/GetGroundCoordRequest")
(setf (get ground_projection::GetGroundCoordRequest :definition-)
      "

duckietown_msgs/Vector2D normalized_uv

================================================================================
MSG: duckietown_msgs/Vector2D
float32 x
float32 y
---

geometry_msgs/Point gp


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get ground_projection::GetGroundCoordResponse :md5sum-) "556869c22d708d07f00608b52f833756")
(setf (get ground_projection::GetGroundCoordResponse :datatype-) "ground_projection/GetGroundCoordResponse")
(setf (get ground_projection::GetGroundCoordResponse :definition-)
      "

duckietown_msgs/Vector2D normalized_uv

================================================================================
MSG: duckietown_msgs/Vector2D
float32 x
float32 y
---

geometry_msgs/Point gp


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :ground_projection/GetGroundCoord "556869c22d708d07f00608b52f833756")


