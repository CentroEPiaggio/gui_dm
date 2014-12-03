#ifndef DUAL_MANIPULATION_GUI_H
#define DUAL_MANIPULATION_GUI_H

#include <QWidget>
#include <QSplitter>
#include <cstdlib>
#include <widgets/camera_widget.h>
#include <widgets/render_widget.h>
#include <widgets/state_machine_widget.h>
#include <widgets/state_widget.h>
#include <widgets/control_widget.h>

class dual_manipulation_gui: public QWidget
{
Q_OBJECT
public:
  dual_manipulation_gui();
  ~dual_manipulation_gui();

private:
  
  render_widget render;
  camera_widget camera;
  state_machine_widget state_machine;
  state_widget state;
  control_widget control;

  QSplitter main_layout;
  
  QSplitter visualization_layout;
  
  QSplitter control_layout;
  
  QSplitter state_layout;
};

#endif // DUAL_MANIPULATION_GUI_H
