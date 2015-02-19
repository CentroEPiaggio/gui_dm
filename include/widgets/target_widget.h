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
#include "geometry_msgs/PointStamped.h"
#include "dual_manipulation_shared/gui_target_service.h"
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>

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

  ros::ServiceServer gui_target_service;
  bool gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request &req, dual_manipulation_shared::gui_target_service::Response &res);
  ros::Subscriber sub;
  bool target_ready=false;
  void clicked_point(const geometry_msgs::PointStampedPtr& point);
  void update_coords();
  void publish_marker();

  ros::Subscriber sub_im, im_sub_fb;
  ros::Publisher pub_target;
  interactive_markers::InteractiveMarkerServer* server;
  visualization_msgs::InteractiveMarker* int_marker;
  void update_position(const visualization_msgs::Marker &marker_);
  visualization_msgs::Marker target_marker;
  void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback);
};

#endif // TARGET_WIDGET_H