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

class control_widget: public QWidget
{
Q_OBJECT
public:
  control_widget(state_machine_widget* smw_);
  ~control_widget();

private Q_SLOTS:
  void on_command_button_clicked(const int& id);
  void on_stop_robot_button_clicked();
  void on_home_robot_button_clicked();

private:
  QSignalMapper signalMapper;
  QGridLayout main_layout;

  QPushButton home_robot_button;
  QPushButton stop_robot_button;
  
  ros::Publisher left_arm_stop,right_arm_stop;

  std::map<int,QPushButton*> map_button;

  ros::NodeHandle n;
  ros::ServiceClient client;
  ros::ServiceClient ik_client;
  dual_manipulation_shared::state_manager_service srv;
  dual_manipulation_shared::ik_service ik_srv;

  state_machine_widget* smw;
};

#endif // CONTROL_WIDGET_H