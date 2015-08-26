#include <widgets/message_widget.h>

message_widget::message_widget()
{
    text.setMinimumSize(10,30);

    main_layout.addWidget(&text);

    setLayout(&main_layout);
}

void message_widget::change_text(std::string new_text, Qt::GlobalColor color)
{
    QPalette *palette = new QPalette();
    palette->setColor(QPalette::Text,color);
    text.setPalette(*palette);
    
    text.setText(QString::fromStdString(new_text));
}

void message_widget::info_message(std::string msg)
{
    change_text(msg);
}

void message_widget::cool_message(std::string msg)
{
    change_text(msg,Qt::blue);
}

void message_widget::warning_message(std::string msg)
{
    change_text(msg,Qt::yellow);
}

void message_widget::error_message(std::string msg)
{
    change_text(msg,Qt::red);
}

message_widget::~message_widget()
{

}