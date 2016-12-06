#ifndef CONTROL_WIDGET_H
#define CONTROL_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <cstdlib>
#include <QSignalMapper>
#include <map>
#include "ros/ros.h"
#include "dual_manipulation_shared/state_manager_service.h"
#include "dual_manipulation_shared/ik_service.h"
#include "widgets/state_machine_widget.h"
#include "widgets/message_widget.h"
#include "widgets/target_widget.h"

class control_widget: public QWidget
{
Q_OBJECT
public:
  control_widget(std::vector<std::string> ns_list, message_widget* message_=NULL, target_widget* target_=NULL);
  ~control_widget();

private Q_SLOTS:
  void on_command_button_clicked(const int& id);
  void on_stop_robot_button_clicked();
  void on_start_robot_button_clicked();
  void on_home_robot_button_clicked();
  void on_quick_button_clicked();

private:
  message_widget* message;
  target_widget* target;
  QSignalMapper signalMapper;
  QGridLayout main_layout;

  QPushButton home_robot_button;
  QPushButton start_robot_button;
  QPushButton stop_robot_button;
  QPushButton quick_button;
  
  ros::Publisher left_arm_stop,right_arm_stop;
  // namespaces for the various robots
  std::vector<std::string> robot_namespaces;
  // safety publishers
  std::vector<ros::Publisher> stop_publishers;
  std::vector<ros::Publisher> emergency_event_publishers;
  // set of controllers to stop/start in case of safety events
  std::vector<std::string> regular_controllers, emergency_handling_controllers;
  std::string switch_controller_service;

  std::map<int,QPushButton*> map_button;

  ros::NodeHandle n;
  std::vector<ros::ServiceClient> clients;
  std::vector<ros::ServiceClient> ik_clients;
  dual_manipulation_shared::state_manager_service srv;
  dual_manipulation_shared::ik_service ik_srv;
};

#endif // CONTROL_WIDGET_H