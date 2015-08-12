#ifndef STATE_MACHINE_WIDGET_H
#define STATE_MACHINE_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <cstdlib>
#include "image_widget.h"
#include <QTimer>
#include <QLabel>
#include <std_msgs/String.h>

class state_machine_widget: public Viewer
{
Q_OBJECT
public:
  state_machine_widget();
  ~state_machine_widget();

  void paintEvent(QPaintEvent* event);
  
private:
  void paintArrow(const QPointF q1, const QPointF q2);
  QGridLayout main_layout;
  QLabel* label;
  QGridLayout* label_layout;
  QTimer timer;
  std::map<std::string, QGraphicsEllipseItem*> states;  
  std::string current_state;
  void stateCallback(const std_msgs::String::ConstPtr & msg);
  
private Q_SLOTS:
    void save();
};

#endif // STATE_MACHINE_WIDGET_H