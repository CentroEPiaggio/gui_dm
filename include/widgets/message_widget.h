#ifndef MESSAGE_WIDGET_H
#define MESSAGE_WIDGET_H

#include <QWidget>
#include <QLineEdit>
#include <QVBoxLayout>

class message_widget: public QWidget
{
public:
    message_widget();
    ~message_widget();

    void info_message(std::string msg);
    void cool_message(std::string msg);
    void warning_message(std::string msg);
    void error_message(std::string msg);

private:
    void change_text(std::string new_text,Qt::GlobalColor color=Qt::black);

    QVBoxLayout main_layout;
    QLineEdit text;
};

#endif //MESSAGE_WIDGET_H