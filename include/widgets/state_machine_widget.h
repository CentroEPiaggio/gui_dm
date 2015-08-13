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

struct Arrow
{
    QGraphicsLineItem *left, *right, *perpendicular;
    QGraphicsPathItem *main;
};

class state_machine_widget: public Viewer
{
Q_OBJECT
public:
  state_machine_widget();
  ~state_machine_widget();

  void paintEvent(QPaintEvent* event);
  
private:
  bool paintArrow(const QPointF q1, const QPointF q2, Arrow& created_arrow);
  QGridLayout main_layout;
  QLabel* label;
  QGridLayout* label_layout;
  QTimer timer;
  std::map<std::string, QGraphicsEllipseItem*> states;  
  std::string current_state;
  std::vector< std::tuple< std::string, std::pair< std::string, bool >, std::string > > table;
  void stateCallback(const std_msgs::String::ConstPtr & msg);
  std::map<std::string,std::map<std::string,Arrow>> arrows;
  std::map<std::string,std::string> type_to_state;
  std::map<std::string,std::string> state_to_visual;
  
private Q_SLOTS:
    void save();
  void moveArrow(Arrow& second, std::string source, std::string target);
};

#endif // STATE_MACHINE_WIDGET_H