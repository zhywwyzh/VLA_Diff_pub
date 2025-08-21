;; Auto-generated. Do not edit!


(when (boundp 'traj_utils::drone_state_to_steamdeck)
  (if (not (find-package "TRAJ_UTILS"))
    (make-package "TRAJ_UTILS"))
  (shadow 'drone_state_to_steamdeck (find-package "TRAJ_UTILS")))
(unless (find-package "TRAJ_UTILS::DRONE_STATE_TO_STEAMDECK")
  (make-package "TRAJ_UTILS::DRONE_STATE_TO_STEAMDECK"))

(in-package "ROS")
;;//! \htmlinclude drone_state_to_steamdeck.msg.html


(defclass traj_utils::drone_state_to_steamdeck
  :super ros::object
  :slots (_drone_id _start_time _current_node_state _debug_info ))

(defmethod traj_utils::drone_state_to_steamdeck
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:current_node_state __current_node_state) 0)
    ((:debug_info __debug_info) "")
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _start_time __start_time)
   (setq _current_node_state (round __current_node_state))
   (setq _debug_info (string __debug_info))
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
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
    ;; time _start_time
    8
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
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
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
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int8 _current_node_state
     (setq _current_node_state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _current_node_state 127) (setq _current_node_state (- _current_node_state 256)))
   ;; string _debug_info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _debug_info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get traj_utils::drone_state_to_steamdeck :md5sum-) "d799a4895f8a152d4ada9c7313ecec9b")
(setf (get traj_utils::drone_state_to_steamdeck :datatype-) "traj_utils/drone_state_to_steamdeck")
(setf (get traj_utils::drone_state_to_steamdeck :definition-)
      "int8 drone_id
# 1: baseFail (vins_odom too low or no odom)
# 2: basepreparing
# 3: baseReady
# 4: px4_take_off
# 5: command
# 6: px4_land
# 7: auto_hover
# 8: locatizationError (optical_flow)
# 9: crash (cmd ---> stopPropeller) 
# 10: drone_node_only
# 11: timeAlignFail
time start_time
int8 current_node_state
string debug_info
")



(provide :traj_utils/drone_state_to_steamdeck "d799a4895f8a152d4ada9c7313ecec9b")


