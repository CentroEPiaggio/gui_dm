#ifndef DUAL_MANIPULATION_GUI_H
#define DUAL_MANIPULATION_GUI_H

#include <QWidget>
#include <QSplitter>
#include <QSocketNotifier>
#include <cstdlib>
#include <widgets/camera_widget.h>
#include <widgets/render_3d_widget.h>
#include <widgets/state_machine_widget.h>
#include <widgets/graph_widget.h>
#include <widgets/state_widget.h>
#include <widgets/control_widget.h>
#include <widgets/target_widget.h>
#include <widgets/message_widget.h>
#include <XmlRpcValue.h>
#include "ros/ros.h"

class dual_manipulation_gui: public QWidget
{
Q_OBJECT
public:
  dual_manipulation_gui();
  ~dual_manipulation_gui();

  //Unix signal handlers
  static void intSignalHandler(int);
protected:
    void closeEvent(QCloseEvent *event);
public Q_SLOTS:
    void handleSigInt();
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
  message_widget message;
  control_widget control;
  target_widget* target;

  QSplitter main_layout;
  
  QSplitter visualization_layout;
  
  QSplitter control_layout;
  
  QSplitter message_layout;
  
  QSplitter state_layout;
  
  static int sigintFd[2];
  QSocketNotifier* snInt;

  std::vector<std::string> ns_list;
};

#endif // DUAL_MANIPULATION_GUI_H
