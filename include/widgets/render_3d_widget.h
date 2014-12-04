#ifndef RENDER_3D_WIDGET_H
#define RENDER_3D_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <cstdlib>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>

class render_3d_widget: public QWidget
{
Q_OBJECT
public:
  render_3d_widget();
  ~render_3d_widget();

private:

  QGridLayout main_layout;
  
  rviz::RenderPanel render_panel;
  rviz::VisualizationManager visualization_manager;
  
};

#endif // RENDER_3D_WIDGET_H