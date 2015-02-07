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

class control_widget: public QWidget
{
Q_OBJECT
public:
  control_widget();
  ~control_widget();

private Q_SLOTS:
  void on_command_button_clicked(const int& id);

private:
  QSignalMapper signalMapper;
  QGridLayout main_layout;

  std::map<int,QPushButton*> map_button;

  ros::NodeHandle n;
  ros::ServiceClient client;
  dual_manipulation_shared::state_manager_service srv;
};

#endif // CONTROL_WIDGET_H