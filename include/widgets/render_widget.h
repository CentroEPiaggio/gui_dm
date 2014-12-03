#ifndef RENDER_WIDGET_H
#define RENDER_WIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <cstdlib>

class render_widget: public QWidget
{
Q_OBJECT
public:
  render_widget();
  ~render_widget();

private:

  QGridLayout main_layout;

};

#endif // RENDER_WIDGET_H