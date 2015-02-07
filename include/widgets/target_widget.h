#ifndef TARGET_WIDGET_H
#define TARGET_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <cstdlib>
#include <QSignalMapper>
#include <map>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

class target_widget: public QWidget
{
Q_OBJECT
public:
  target_widget();
  ~target_widget();

private Q_SLOTS:
  void on_coord_edit_changed(const int& id);
  void on_set_target_clicked();

private:
  QSignalMapper signalMapper;
  QGridLayout main_layout;

  std::map<int,QLineEdit*> coord_map;
  std::vector<QLabel*> coord_label;
  std::vector<QHBoxLayout*> coord_sublayout;

  QPushButton set_target_button;
  geometry_msgs::Pose target_pose;
  ros::NodeHandle n;
  ros::Publisher target_pub;
};

#endif // TARGET_WIDGET_H