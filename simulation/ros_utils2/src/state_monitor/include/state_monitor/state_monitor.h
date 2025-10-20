#ifndef _STATE_MONITOR_H
#define _STATE_MONITOR_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <std_msgs/Int32.h>
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <ftxui/screen/string.hpp>
#include <ftxui/screen/color.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include "ftxui/dom/node.hpp"      // for Render
#include "ftxui/screen/color.hpp"  // for ftxui

enum class COMMAND_TYPE: int { 
  WAIT=0, STOP, GO, NEXT, GO_ORIGIN, AGAIN, 
  EMERGENCY_STOP, RESTART, GET_PRE, REPLAN };
enum class VLA_STATE: int { 
  INIT=0, WAIT_FOR_MISSION, PLAN, PUBLISH, WAIT_ACTION_FINISH, 
  FINISH, STOP, ERROR, GO_ORIGIN, REPLY_MLLM, EGO_FINISH };

class StateMonitor {
public:
  StateMonitor(){};
  ~StateMonitor(){};
  void init(ros::NodeHandle& nh);
  void updateDisplay();
  void drawTUI();

private:
  void clearTerminal();
  void vlaStateCallback(const std_msgs::Int32::ConstPtr& msg);
  void commandTypeCallback(const std_msgs::Int32::ConstPtr& msg);
  void commandContentCallback(const std_msgs::String::ConstPtr& msg);
  void finishMissionCallback(const std_msgs::Bool::ConstPtr& msg);
  void finishCommandCallback(const std_msgs::Bool::ConstPtr& msg);
  std::wstring toWStringWithPrecision(float value, int precision);
  ros::Subscriber vla_state_sub_, command_type_sub_, command_content_sub_, finish_mission_sub_, finish_command_sub_;

  COMMAND_TYPE command_type_{COMMAND_TYPE::WAIT};
  VLA_STATE vla_state_{VLA_STATE::INIT};

  std::vector<std::string> command_content_;
  bool finish_mission_;
  bool finish_command_;
};

#endif
