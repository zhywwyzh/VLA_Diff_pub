;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::node_only_drone_node)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'node_only_drone_node (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::NODE_ONLY_DRONE_NODE")
  (make-package "TRAJ_UTILS::NODE_ONLY_DRONE_NODE"))

(in-package "ROS")
;;//! \htmlinclude node_only_drone_node.msg.html


(defclass traj_utils::node_only_drone_node
  :super ros::object
  :slots (_drone_id _node_already_reastart ))

(defmethod traj_utils::node_only_drone_node
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:node_already_reastart __node_already_reastart) 0)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _node_already_reastart (round __node_already_reastart))
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:node_already_reastart
   (&optional __node_already_reastart)
   (if __node_already_reastart (setq _node_already_reastart __node_already_reastart)) _node_already_reastart)
  (:serialization-length
   ()
   (+
    ;; int8 _drone_id
    1
    ;; int8 _node_already_reastart
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _drone_id
       (write-byte _drone_id s)
     ;; int8 _node_already_reastart
       (write-byte _node_already_reastart s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _drone_id
     (setq _drone_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _drone_id 127) (setq _drone_id (- _drone_id 256)))
   ;; int8 _node_already_reastart
     (setq _node_already_reastart (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _node_already_reastart 127) (setq _node_already_reastart (- _node_already_reastart 256)))
   ;;
   self)
  )

(setf (get traj_utils::node_only_drone_node :md5sum-) "33299fffa442f68d79e075c04bbd062c")
(setf (get traj_utils::node_only_drone_node :datatype-) "traj_utils/node_only_drone_node")
(setf (get traj_utils::node_only_drone_node :definition-)
      "int8 drone_id
int8 node_already_reastart
")



(provide :traj_utils/node_only_drone_node "33299fffa442f68d79e075c04bbd062c")


