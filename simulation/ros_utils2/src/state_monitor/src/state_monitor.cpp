#include <state_monitor/state_monitor.h>
#include <chrono>
#include <string>
#include <thread>
#include <regex>
#include <locale>
#include <codecvt>
#include <iostream>

using namespace ftxui;
using namespace std::chrono_literals;

void StateMonitor::init(ros::NodeHandle& nh)
{
  vla_state_sub_ = nh.subscribe("monitor/vla_state", 10, &StateMonitor::vlaStateCallback, this);
  command_type_sub_ = nh.subscribe("monitor/command_type", 10, &StateMonitor::commandTypeCallback, this);
  command_content_sub_ = nh.subscribe("monitor/command_content", 10, &StateMonitor::commandContentCallback, this);
  vla_state_ = VLA_STATE::INIT;
  command_type_ = COMMAND_TYPE::WAIT;

  clearTerminal();
}

auto create_mode_box = [](const std::wstring& label, const std::wstring& selected_state) {
  if (selected_state.empty()) {
    return ftxui::text(label) | ftxui::border | ftxui::color(ftxui::Color::White);
  }
  else {
    return ftxui::text(label) | ftxui::border | ftxui::color(ftxui::Color::Green);
  }
};

void StateMonitor::clearTerminal()
{
  constexpr const char* CLEAR_SCREEN = "\033[2J\033[1;1H";
  printf("%s", CLEAR_SCREEN);
}

std::wstring convertToWstring(const std::string& str)
{
    std::wcin.imbue(std::locale("en_US.UTF-8"));
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    return converter.from_bytes(str);
}

void StateMonitor::drawTUI()
{
    auto document = ftxui::vbox({
        ftxui::separator(),
        ftxui::hbox({
            ftxui::text(L"Here is the state of the VLA") | ftxui::bold | ftxui::color(ftxui::Color::Cyan) | ftxui::align_right
        }) | ftxui::center,
        ftxui::separator(),
        ftxui::hbox({
            ftxui::text(L"VLA_STATE: ") | ftxui::bold | ftxui::color(ftxui::Color::Cyan) | ftxui::align_right
        }) | ftxui::center,
        ftxui::separator(),
        ftxui::hbox({
            create_mode_box(L"INIT", vla_state_ == VLA_STATE::INIT ? L"INIT" : L""),
            ftxui::separator(),
            create_mode_box(L"WAIT_FOR_MISSION", vla_state_ == VLA_STATE::WAIT_FOR_MISSION ? L"WAIT_FOR_MISSION" : L""),
            ftxui::separator(),
            create_mode_box(L"PLAN", vla_state_ == VLA_STATE::PLAN ? L"PLAN" : L""),
            ftxui::separator(),
            create_mode_box(L"PUBLISH", vla_state_ == VLA_STATE::PUBLISH ? L"PUBLISH" : L""),
            ftxui::separator(),
            create_mode_box(L"WAIT_ACTION_FINISH", vla_state_ == VLA_STATE::WAIT_ACTION_FINISH ? L"WAIT_ACTION_FINISH" : L""),
            ftxui::separator(),
            create_mode_box(L"FINISH", vla_state_ == VLA_STATE::FINISH ? L"FINISH" : L""),
            ftxui::separator(),
            create_mode_box(L"STOP", vla_state_ == VLA_STATE::STOP ? L"STOP" : L""),
            ftxui::separator(),
            create_mode_box(L"ERROR", vla_state_ == VLA_STATE::ERROR ? L"ERROR" : L""),
            ftxui::separator(),
            create_mode_box(L"GO_ORIGIN", vla_state_ == VLA_STATE::GO_ORIGIN ? L"GO_ORIGIN" : L""),
            ftxui::separator(),
            create_mode_box(L"REPLY_MLLM", vla_state_ == VLA_STATE::REPLY_MLLM ? L"REPLY_MLLM" : L""),
            ftxui::separator(),
            create_mode_box(L"EGO_FINISH", vla_state_ == VLA_STATE::EGO_FINISH ? L"EGO_FINISH" : L""),
        }) | ftxui::center,
        ftxui::separator(),
        ftxui::hbox({
            ftxui::text(L"COMMAND_TYPE: ") | ftxui::bold | ftxui::color(ftxui::Color::Cyan) | ftxui::align_right
        }) | ftxui::center,
        ftxui::separator(),
        ftxui::hbox({
            create_mode_box(L"WAIT", command_type_ == COMMAND_TYPE::WAIT ? L"WAIT" : L""),
            ftxui::separator(),
            create_mode_box(L"STOP", command_type_ == COMMAND_TYPE::STOP ? L"STOP" : L""),
            ftxui::separator(),
            create_mode_box(L"GO", command_type_ == COMMAND_TYPE::GO ? L"GO" : L""),
            ftxui::separator(),
            create_mode_box(L"NEXT", command_type_ == COMMAND_TYPE::NEXT ? L"NEXT" : L""),
            ftxui::separator(),
            create_mode_box(L"GO_ORIGIN", command_type_ == COMMAND_TYPE::GO_ORIGIN ? L"GO_ORIGIN" : L""),
            ftxui::separator(),
            create_mode_box(L"AGAIN", command_type_ == COMMAND_TYPE::AGAIN ? L"AGAIN" : L""),
            ftxui::separator(),
            create_mode_box(L"EMERGENCY_STOP", command_type_ == COMMAND_TYPE::EMERGENCY_STOP ? L"EMERGENCY_STOP" : L""),
            ftxui::separator(),
            create_mode_box(L"RESTART", command_type_ == COMMAND_TYPE::RESTART ? L"RESTART" : L""),
            ftxui::separator(),
            create_mode_box(L"GET_PRE", command_type_ == COMMAND_TYPE::GET_PRE ? L"GET_PRE" : L""),
            ftxui::separator(),
            create_mode_box(L"REPLAN", command_type_ == COMMAND_TYPE::REPLAN ? L"REPLAN" : L"")
        }) | ftxui::center,
        ftxui::separator(),
        ftxui::hbox({
            ftxui::text(L"COMMAND_CONTENT: ") | ftxui::bold | ftxui::color(ftxui::Color::Cyan) | ftxui::align_right
        }) | ftxui::center,
        ftxui::separator(),
        ftxui::hbox({
            ftxui::text(command_content_.empty() ? L"None" : convertToWstring(command_content_.back())) | 
                ftxui::bold | ftxui::color(ftxui::Color::YellowLight) | ftxui::border
        }) | ftxui::center,
    }) | ftxui::center;

    auto screen = ftxui::Screen::Create(ftxui::Dimension::Full(), ftxui::Dimension::Fit(document));
    Render(screen, document);
    clearTerminal();
    screen.Print();
}

void StateMonitor::vlaStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
  vla_state_ = static_cast<VLA_STATE>(msg->data);
}

void StateMonitor::commandTypeCallback(const std_msgs::Int32::ConstPtr& msg)
{
  command_type_ = static_cast<COMMAND_TYPE>(msg->data);
}

void StateMonitor::commandContentCallback(const std_msgs::String::ConstPtr& msg)
{
  command_content_.push_back(msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_monitor_node");
  ros::NodeHandle nh;
  StateMonitor state_monitor_screen;
  state_monitor_screen.init(nh);

  ros::Rate loop_rate(20); // 20 Hz
  while (ros::ok()) {
    state_monitor_screen.drawTUI();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}