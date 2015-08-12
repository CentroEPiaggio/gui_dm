#ifndef GRAPH_WIDGET_H
#define GRAPH_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <cstdlib>
#include <QTimer>
#include <QLabel>

class graph_widget: public QWidget
{
Q_OBJECT
public:
    graph_widget();
    ~graph_widget();

private:

  QGridLayout main_layout;

  QLabel *label;
  QGridLayout *label_layout, *label_layout1;
};

#endif // STATE_MACHINE_WIDGET_H