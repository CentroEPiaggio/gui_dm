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
#include <QComboBox>
#include <map>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "dual_manipulation_shared/gui_target_service.h"
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include "dual_manipulation_shared/databasemapper.h"

class target_widget: public QWidget
{
Q_OBJECT
public:
  target_widget();
  ~target_widget();

private Q_SLOTS:
  void on_source_coord_edit_changed(const int& id);
  void on_target_coord_edit_changed(const int& id);
  void on_set_target_clicked();
  void publish_marker();
  void on_object_changed();

private:
  QSignalMapper source_signalMapper, target_signalMapper;
  QGridLayout main_layout;

  QLabel source_label, target_label;
  
  std::map<int,QLineEdit*> source_coord_map;
  std::map<int,QLineEdit*> target_coord_map;
  std::vector<QLabel*> source_coord_label;
  std::vector<QLabel*> target_coord_label;
  std::vector<QHBoxLayout*> source_coord_sublayout;
  std::vector<QHBoxLayout*> target_coord_sublayout;

  QComboBox clicking_pose;
  QComboBox object_selection;
  QPushButton publish_button;
  QPushButton set_target_button;
  geometry_msgs::Pose source_pose, target_pose;
  ros::NodeHandle n;

  ros::ServiceServer gui_target_service;
  bool gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request &req, dual_manipulation_shared::gui_target_service::Response &res);
  ros::Subscriber sub;
  bool target_ready=false;
  void clicked_point(const geometry_msgs::PointStampedPtr& point);
  void update_coords(std::map<int,QLineEdit*> coord_map, geometry_msgs::Pose pose);
  int obj_id_;

  ros::Subscriber sub_im, im_sub_fb;
  ros::Publisher pub_target;
  interactive_markers::InteractiveMarkerServer* server;
  visualization_msgs::InteractiveMarker* int_marker;
  void update_position(const visualization_msgs::Marker &marker_);
  visualization_msgs::Marker source_marker, target_marker;
  void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback);

  databaseMapper db_mapper;
  void update_mesh_resources();
};

#endif // TARGET_WIDGET_H