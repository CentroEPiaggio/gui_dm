#ifndef DUAL_MANIPULATION_GUI_H
#define DUAL_MANIPULATION_GUI_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <cstdlib>

class dual_manipulation_gui: public QWidget
{
Q_OBJECT
public:
  dual_manipulation_gui();
  ~dual_manipulation_gui();

private:

  QVBoxLayout main_layout;
  
  QHBoxLayout visualization_layout;
  
  QHBoxLayout control_layout;
  
  QHBoxLayout commands_layout;
};

#endif // DUAL_MANIPULATION_GUI_H
