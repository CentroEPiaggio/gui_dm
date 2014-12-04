#ifndef CONTROL_WIDGET_H
#define CONTROL_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <cstdlib>

class control_widget: public QWidget
{
Q_OBJECT
public:
  control_widget();
  ~control_widget();

private:

  QGridLayout main_layout;

  QPushButton example_button;
};

#endif // CONTROL_WIDGET_H