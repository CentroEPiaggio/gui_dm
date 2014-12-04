#ifndef CAMERA_WIDGET_H
#define CAMERA_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <cstdlib>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

class camera_widget: public QWidget
{
Q_OBJECT
public:
  camera_widget();
  ~camera_widget();

private:

  QGridLayout main_layout;

  rviz::RenderPanel render_panel;
  rviz::VisualizationManager visualization_manager;
  rviz::Display* camera_display;

};

#endif // CAMERA_WIDGET_H