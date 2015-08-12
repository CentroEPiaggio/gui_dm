#ifndef DUAL_MANIPULATION_VISUALIZATION_GUI_H
#define DUAL_MANIPULATION_VISUALIZATION_GUI_H

#include <widgets/state_widget.h>

class dual_manipulation_visualization_gui: public QWidget
{
Q_OBJECT
public:
  dual_manipulation_visualization_gui();
  ~dual_manipulation_visualization_gui();

private:
  QGridLayout main_layout;
  QGridLayout* label_layout, *label_layout1;
};

#endif // DUAL_MANIPULATION_VISUALIZATION_GUI_H
