#ifndef STATE_MACHINE_WIDGET_H
#define STATE_MACHINE_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <cstdlib>
#include <QTimer>
#include <QLabel>

class state_machine_widget: public QWidget
{
Q_OBJECT
public:
  state_machine_widget();
  ~state_machine_widget();
  void start_timer();
  void stop_timer();

private Q_SLOTS:
  void timer_body();

private:

  QGridLayout main_layout;
  QTimer timer;

  std::string img_path;
  QLabel* label;
  QGridLayout* label_layout;
};

#endif // STATE_MACHINE_WIDGET_H