#ifndef STOP_WINDOW_H
#define STOP_WINDOW_H

#include "ros/ros.h"
#include <widgets/state_widget.h>
#include <QPushButton>
#include "dual_manipulation_shared/ik_service.h"

class stop_window: public QWidget
{
Q_OBJECT
public:
  stop_window();
  ~stop_window();

private Q_SLOTS:
  void on_stop_robot_button_clicked();

private:
  QGridLayout main_layout;
  QPushButton stop_robot_button;
  ros::NodeHandle n;
  ros::Publisher left_arm_stop,right_arm_stop;
};

#endif // STOP_WINDOW_H