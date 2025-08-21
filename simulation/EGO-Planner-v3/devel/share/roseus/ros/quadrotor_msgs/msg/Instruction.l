;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_msgs::Instruction)
  (if (not (find-package "QUADROTOR_MSGS"))
    (make-package "QUADROTOR_MSGS"))
  (shadow 'Instruction (find-package "QUADROTOR_MSGS")))
(unless (find-package "QUADROTOR_MSGS::INSTRUCTION")
  (make-package "QUADROTOR_MSGS::INSTRUCTION"))

(in-package "ROS")
;;//! \htmlinclude Instruction.msg.html


(intern "*TURN_GOAL*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*TURN_GOAL* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*TURN_GOAL* 1)
(intern "*TURN_EXPLORE*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*TURN_EXPLORE* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*TURN_EXPLORE* 2)
(intern "*TURN_PATROL*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*TURN_PATROL* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*TURN_PATROL* 3)
(intern "*TURN_HIT*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*TURN_HIT* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*TURN_HIT* 4)
(intern "*GO_HOME*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*GO_HOME* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*GO_HOME* 5)
(intern "*TURN_EGO_GOAL*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*TURN_EGO_GOAL* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*TURN_EGO_GOAL* 6)
(intern "*SHARE_MAP*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*SHARE_MAP* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*SHARE_MAP* 7)
(intern "*RESET_EXPLORE_AREA*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*RESET_EXPLORE_AREA* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*RESET_EXPLORE_AREA* 8)
(intern "*MAP_MERGE_REQUEST*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*MAP_MERGE_REQUEST* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*MAP_MERGE_REQUEST* 9)
(intern "*MAP_CIRCULATE*" (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(shadow '*MAP_CIRCULATE* (find-package "QUADROTOR_MSGS::INSTRUCTION"))
(defconstant quadrotor_msgs::Instruction::*MAP_CIRCULATE* 10)

(defun quadrotor_msgs::Instruction-to-symbol (const)
  (cond
        ((= const 1) 'quadrotor_msgs::Instruction::*TURN_GOAL*)
        ((= const 2) 'quadrotor_msgs::Instruction::*TURN_EXPLORE*)
        ((= const 3) 'quadrotor_msgs::Instruction::*TURN_PATROL*)
        ((= const 4) 'quadrotor_msgs::Instruction::*TURN_HIT*)
        ((= const 5) 'quadrotor_msgs::Instruction::*GO_HOME*)
        ((= const 6) 'quadrotor_msgs::Instruction::*TURN_EGO_GOAL*)
        ((= const 7) 'quadrotor_msgs::Instruction::*SHARE_MAP*)
        ((= const 8) 'quadrotor_msgs::Instruction::*RESET_EXPLORE_AREA*)
        ((= const 9) 'quadrotor_msgs::Instruction::*MAP_MERGE_REQUEST*)
        ((= const 10) 'quadrotor_msgs::Instruction::*MAP_CIRCULATE*)
        (t nil)))

(defclass quadrotor_msgs::Instruction
  :super ros::object
  :slots (_robot_id _instruction_type _target_position _src_drone_ids _tar_drone_ids ))

(defmethod quadrotor_msgs::Instruction
  (:init
   (&key
    ((:robot_id __robot_id) 0)
    ((:instruction_type __instruction_type) 0)
    ((:target_position __target_position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:src_drone_ids __src_drone_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tar_drone_ids __tar_drone_ids) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _robot_id (round __robot_id))
   (setq _instruction_type (round __instruction_type))
   (setq _target_position __target_position)
   (setq _src_drone_ids __src_drone_ids)
   (setq _tar_drone_ids __tar_drone_ids)
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:instruction_type
   (&optional __instruction_type)
   (if __instruction_type (setq _instruction_type __instruction_type)) _instruction_type)
  (:target_position
   (&optional __target_position)
   (if __target_position (setq _target_position __target_position)) _target_position)
  (:src_drone_ids
   (&optional __src_drone_ids)
   (if __src_drone_ids (setq _src_drone_ids __src_drone_ids)) _src_drone_ids)
  (:tar_drone_ids
   (&optional __tar_drone_ids)
   (if __tar_drone_ids (setq _tar_drone_ids __tar_drone_ids)) _tar_drone_ids)
  (:serialization-length
   ()
   (+
    ;; uint8 _robot_id
    1
    ;; uint8 _instruction_type
    1
    ;; float32[3] _target_position
    (* 4    3)
    ;; uint16[] _src_drone_ids
    (* 2    (length _src_drone_ids)) 4
    ;; uint16[] _tar_drone_ids
    (* 2    (length _tar_drone_ids)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _robot_id
       (write-byte _robot_id s)
     ;; uint8 _instruction_type
       (write-byte _instruction_type s)
     ;; float32[3] _target_position
     (dotimes (i 3)
       (sys::poke (elt _target_position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint16[] _src_drone_ids
     (write-long (length _src_drone_ids) s)
     (dotimes (i (length _src_drone_ids))
       (write-word (elt _src_drone_ids i) s)
       )
     ;; uint16[] _tar_drone_ids
     (write-long (length _tar_drone_ids) s)
     (dotimes (i (length _tar_drone_ids))
       (write-word (elt _tar_drone_ids i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _robot_id
     (setq _robot_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _instruction_type
     (setq _instruction_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32[3] _target_position
   (dotimes (i (length _target_position))
     (setf (elt _target_position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint16[] _src_drone_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _src_drone_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _src_drone_ids i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _tar_drone_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tar_drone_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _tar_drone_ids i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get quadrotor_msgs::Instruction :md5sum-) "6740b6e4e6245eb3348061673116e30f")
(setf (get quadrotor_msgs::Instruction :datatype-) "quadrotor_msgs/Instruction")
(setf (get quadrotor_msgs::Instruction :definition-)
      "uint8 robot_id
uint8 instruction_type
float32[3] target_position

# map merge request
uint16[] src_drone_ids
uint16[] tar_drone_ids

#definations for instruction_type
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
")



(provide :quadrotor_msgs/Instruction "6740b6e4e6245eb3348061673116e30f")


