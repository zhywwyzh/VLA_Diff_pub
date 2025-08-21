;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::InstructionResMsg)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'InstructionResMsg (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG")
  (make-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))

(in-package "ROS")
;;//! \htmlinclude InstructionResMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*TURN_GOAL*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*TURN_GOAL* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*TURN_GOAL* 1)
(intern "*TURN_EXPLORE*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*TURN_EXPLORE* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*TURN_EXPLORE* 2)
(intern "*TURN_PATROL*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*TURN_PATROL* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*TURN_PATROL* 3)
(intern "*TURN_HIT*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*TURN_HIT* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*TURN_HIT* 4)
(intern "*GO_HOME*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*GO_HOME* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*GO_HOME* 5)
(intern "*TURN_EGO_GOAL*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*TURN_EGO_GOAL* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*TURN_EGO_GOAL* 6)
(intern "*SHARE_MAP*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*SHARE_MAP* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*SHARE_MAP* 7)
(intern "*RESET_EXPLORE_AREA*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*RESET_EXPLORE_AREA* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*RESET_EXPLORE_AREA* 8)
(intern "*MAP_MERGE_REQUEST*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*MAP_MERGE_REQUEST* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*MAP_MERGE_REQUEST* 9)
(intern "*MAP_CIRCULATE*" (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(shadow '*MAP_CIRCULATE* (find-package "QUADROTOR_MSGS::INSTRUCTIONRESMSG"))
(defconstant quadrotor_msgs::InstructionResMsg::*MAP_CIRCULATE* 10)

(defun quadrotor_msgs::InstructionResMsg-to-symbol (const)
  (cond
        ((= const 1) 'quadrotor_msgs::InstructionResMsg::*TURN_GOAL*)
        ((= const 2) 'quadrotor_msgs::InstructionResMsg::*TURN_EXPLORE*)
        ((= const 3) 'quadrotor_msgs::InstructionResMsg::*TURN_PATROL*)
        ((= const 4) 'quadrotor_msgs::InstructionResMsg::*TURN_HIT*)
        ((= const 5) 'quadrotor_msgs::InstructionResMsg::*GO_HOME*)
        ((= const 6) 'quadrotor_msgs::InstructionResMsg::*TURN_EGO_GOAL*)
        ((= const 7) 'quadrotor_msgs::InstructionResMsg::*SHARE_MAP*)
        ((= const 8) 'quadrotor_msgs::InstructionResMsg::*RESET_EXPLORE_AREA*)
        ((= const 9) 'quadrotor_msgs::InstructionResMsg::*MAP_MERGE_REQUEST*)
        ((= const 10) 'quadrotor_msgs::InstructionResMsg::*MAP_CIRCULATE*)
        (t nil)))

(defclass quadrotor_msgs::InstructionResMsg
  :super ros::object
  :slots (_header _instruction_type _drone_id _is_succeed _tar_drone_id _src_drone_id _failed_drone_id ))

(defmethod quadrotor_msgs::InstructionResMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:instruction_type __instruction_type) 0)
    ((:drone_id __drone_id) 0)
    ((:is_succeed __is_succeed) nil)
    ((:tar_drone_id __tar_drone_id) (make-array 0 :initial-element 0 :element-type :char))
    ((:src_drone_id __src_drone_id) (make-array 0 :initial-element 0 :element-type :char))
    ((:failed_drone_id __failed_drone_id) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _instruction_type (round __instruction_type))
   (setq _drone_id (round __drone_id))
   (setq _is_succeed __is_succeed)
   (setq _tar_drone_id __tar_drone_id)
   (setq _src_drone_id __src_drone_id)
   (setq _failed_drone_id __failed_drone_id)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:instruction_type
   (&optional __instruction_type)
   (if __instruction_type (setq _instruction_type __instruction_type)) _instruction_type)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:is_succeed
   (&optional (__is_succeed :null))
   (if (not (eq __is_succeed :null)) (setq _is_succeed __is_succeed)) _is_succeed)
  (:tar_drone_id
   (&optional __tar_drone_id)
   (if __tar_drone_id (setq _tar_drone_id __tar_drone_id)) _tar_drone_id)
  (:src_drone_id
   (&optional __src_drone_id)
   (if __src_drone_id (setq _src_drone_id __src_drone_id)) _src_drone_id)
  (:failed_drone_id
   (&optional __failed_drone_id)
   (if __failed_drone_id (setq _failed_drone_id __failed_drone_id)) _failed_drone_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _instruction_type
    1
    ;; uint8 _drone_id
    1
    ;; bool _is_succeed
    1
    ;; uint8[] _tar_drone_id
    (* 1    (length _tar_drone_id)) 4
    ;; uint8[] _src_drone_id
    (* 1    (length _src_drone_id)) 4
    ;; uint8[] _failed_drone_id
    (* 1    (length _failed_drone_id)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _instruction_type
       (write-byte _instruction_type s)
     ;; uint8 _drone_id
       (write-byte _drone_id s)
     ;; bool _is_succeed
       (if _is_succeed (write-byte -1 s) (write-byte 0 s))
     ;; uint8[] _tar_drone_id
     (write-long (length _tar_drone_id) s)
     (princ _tar_drone_id s)
     ;; uint8[] _src_drone_id
     (write-long (length _src_drone_id) s)
     (princ _src_drone_id s)
     ;; uint8[] _failed_drone_id
     (write-long (length _failed_drone_id) s)
     (princ _failed_drone_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _instruction_type
     (setq _instruction_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _drone_id
     (setq _drone_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _is_succeed
     (setq _is_succeed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8[] _tar_drone_id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _tar_drone_id (make-array n :element-type :char))
     (replace _tar_drone_id buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _src_drone_id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _src_drone_id (make-array n :element-type :char))
     (replace _src_drone_id buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _failed_drone_id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _failed_drone_id (make-array n :element-type :char))
     (replace _failed_drone_id buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get quadrotor_msgs::InstructionResMsg :md5sum-) "ebdd566ae6eb6d7e30fd1625d7dfda91")
(setf (get quadrotor_msgs::InstructionResMsg :datatype-) "quadrotor_msgs/InstructionResMsg")
(setf (get quadrotor_msgs::InstructionResMsg :definition-)
      "Header header
uint8 instruction_type
uint8 drone_id
bool is_succeed

# -- map merge and share map -- #
uint8[] tar_drone_id
uint8[] src_drone_id
uint8[] failed_drone_id

# -- instruction type defination-- #
uint8 TURN_GOAL = 1
uint8 TURN_EXPLORE = 2
uint8 TURN_PATROL = 3
uint8 TURN_HIT = 4
uint8 GO_HOME = 5
uint8 TURN_EGO_GOAL = 6
uint8 SHARE_MAP = 7
uint8 RESET_EXPLORE_AREA = 8
uint8 MAP_MERGE_REQUEST = 9
uint8 MAP_CIRCULATE = 10

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

")



(provide :quadrotor_msgs/InstructionResMsg "ebdd566ae6eb6d7e30fd1625d7dfda91")


