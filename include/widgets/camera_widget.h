#ifndef CAMERA_WIDGET_H
#define CAMERA_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <cstdlib>

class camera_widget: public QWidget
{
Q_OBJECT
public:
  camera_widget();
  ~camera_widget();

private:

  QGridLayout main_layout;

};

#endif // CAMERA_WIDGET_H