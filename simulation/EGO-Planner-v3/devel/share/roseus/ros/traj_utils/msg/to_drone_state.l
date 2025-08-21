;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::to_drone_state)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'to_drone_state (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::TO_DRONE_STATE")
  (make-package "TRAJ_UTILS::TO_DRONE_STATE"))

(in-package "ROS")
;;//! \htmlinclude to_drone_state.msg.html


(defclass traj_utils::to_drone_state
  :super ros::object
  :slots (_drone_id _current_node_state _debug_info ))

(defmethod traj_utils::to_drone_state
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:current_node_state __current_node_state) 0)
    ((:debug_info __debug_info) "")
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _current_node_state (round __current_node_state))
   (setq _debug_info (string __debug_info))
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:current_node_state
   (&optional __current_node_state)
   (if __current_node_state (setq _current_node_state __current_node_state)) _current_node_state)
  (:debug_info
   (&optional __debug_info)
   (if __debug_info (setq _debug_info __debug_info)) _debug_info)
  (:serialization-length
   ()
   (+
    ;; int8 _drone_id
    1
    ;; int8 _current_node_state
    1
    ;; string _debug_info
    4 (length _debug_info)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _drone_id
       (write-byte _drone_id s)
     ;; int8 _current_node_state
       (write-byte _current_node_state s)
     ;; string _debug_info
       (write-long (length _debug_info) s) (princ _debug_info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _drone_id
     (setq _drone_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _drone_id 127) (setq _drone_id (- _drone_id 256)))
   ;; int8 _current_node_state
     (setq _current_node_state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _current_node_state 127) (setq _current_node_state (- _current_node_state 256)))
   ;; string _debug_info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _debug_info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get traj_utils::to_drone_state :md5sum-) "82065b5642df1c1cef531f1d008cd434")
(setf (get traj_utils::to_drone_state :datatype-) "traj_utils/to_drone_state")
(setf (get traj_utils::to_drone_state :definition-)
      "int8 drone_id
# 4: take_off
# 5: command
# 6: land
int8 current_node_state
string debug_info
")



(provide :traj_utils/to_drone_state "82065b5642df1c1cef531f1d008cd434")


