#ifndef STATE_MACHINE_WIDGET_H
#define STATE_MACHINE_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <cstdlib>

class state_machine_widget: public QWidget
{
Q_OBJECT
public:
  state_machine_widget();
  ~state_machine_widget();

private:

  QGridLayout main_layout;

  QPushButton example_button;
};

#endif // STATE_MACHINE_WIDGET_H