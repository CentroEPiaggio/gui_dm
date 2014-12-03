#ifndef CONTROL_WIDGET_H
#define CONTROL_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <cstdlib>

class control_widget: public QWidget
{
Q_OBJECT
public:
  control_widget();
  ~control_widget();

private:

  QGridLayout main_layout;

};

#endif // CONTROL_WIDGET_H