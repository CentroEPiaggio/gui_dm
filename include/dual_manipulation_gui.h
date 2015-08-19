#ifndef DUAL_MANIPULATION_GUI_H
#define DUAL_MANIPULATION_GUI_H

#include <QWidget>
#include <QSplitter>
#include <cstdlib>
#include <widgets/camera_widget.h>
#include <widgets/render_3d_widget.h>
#include <widgets/state_machine_widget.h>
#include <widgets/graph_widget.h>
#include <widgets/state_widget.h>
#include <widgets/control_widget.h>
#include <widgets/target_widget.h>
#include <XmlRpcValue.h>
#include "ros/ros.h"

class dual_manipulation_gui: public QWidget
{
Q_OBJECT
public:
  dual_manipulation_gui();
  ~dual_manipulation_gui();

protected:
    void closeEvent(QCloseEvent *event);
  
private:
  void parseParameters(XmlRpc::XmlRpcValue& params);
  void readSettings();
  void writeSettings();
  
  XmlRpc::XmlRpcValue gui_params;
  ros::NodeHandle node;
  bool setting_source_position=false;
//   render_3d_widget render;
//   camera_widget camera;
  graph_widget graph;
  state_machine_widget state;
  control_widget control;
  target_widget* target;

  QSplitter main_layout;
  
  QSplitter visualization_layout;
  
  QSplitter control_layout;
  
  QSplitter state_layout;
};

#endif // DUAL_MANIPULATION_GUI_H
