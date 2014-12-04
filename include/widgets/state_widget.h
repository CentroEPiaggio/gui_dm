#ifndef STATE_WIDGET_H
#define STATE_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QTreeWidget>
#include <cstdlib>

class state_widget: public QWidget
{
Q_OBJECT
public:
  state_widget();
  ~state_widget();

private:

  QGridLayout main_layout;
  
  QTreeWidget status_table;

};

#endif // STATE_WIDGET_H