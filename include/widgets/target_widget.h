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
#include <mutex>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "dual_manipulation_shared/gui_target_service.h"
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include "dual_manipulation_shared/databasemapper.h"
#include "tf/transform_broadcaster.h"
#include "dual_manipulation_shared/good_grasp_msg.h"

class target_widget: public QWidget
{
Q_OBJECT
public:
  target_widget(bool setting_source_position_);
  ~target_widget();

private Q_SLOTS:
  void on_source_coord_edit_changed(const int& id);
  void on_target_coord_edit_changed(const int& id);
  void on_set_target_clicked();
  void publish_marker();
  void on_object_changed();

private:
  bool setting_source_position;
  QSignalMapper source_signalMapper, target_signalMapper;
  QGridLayout main_layout;
  ros::Subscriber good_grasp_subscriber;
  ros::Publisher good_grasp_publisher;
  void good_grasp_callback(dual_manipulation_shared::good_grasp_msg msg);

  QLabel source_label, target_label;

  std::mutex callback_mutex;
  
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
  std::string source_id;
  ros::NodeHandle n;

  ros::ServiceServer gui_target_service;
  bool gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request &req, dual_manipulation_shared::gui_target_service::Response &res);
  ros::Subscriber sub;
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
  tf::TransformBroadcaster br;

  std::vector<geometry_msgs::Pose> source_poses;
  std::vector<std::string> source_ids;
  ros::Publisher target_pub;
};

#endif // TARGET_WIDGET_H