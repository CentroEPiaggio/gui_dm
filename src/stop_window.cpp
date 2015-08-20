#include "stop_window.h"
#include "ros/package.h"
#include <std_msgs/String.h>

stop_window::stop_window(): QWidget()
{
    left_arm_stop = n.advertise<std_msgs::String>("/left_arm/emergency_stop",10);
    right_arm_stop = n.advertise<std_msgs::String>("/right_arm/emergency_stop",10);

    QString path_to_package = QString::fromStdString(ros::package::getPath("dual_manipulation_gui"));
    stop_robot_button.setMinimumSize(10,10);
    stop_robot_button.setMaximumSize(1000,1000);
    stop_robot_button.setIcon(QIcon(path_to_package + "/stop.png"));
    stop_robot_button.setIconSize( QSize(stop_robot_button.size().width(), stop_robot_button.size().height() ));
    
    connect(&stop_robot_button,SIGNAL(clicked(bool)), this, SLOT(on_stop_robot_button_clicked()));

    main_layout.addWidget(&stop_robot_button,0,0,Qt::AlignCenter);

    setLayout(&main_layout);
}

void stop_window::on_stop_robot_button_clicked()
{
    ROS_INFO("command STOP to ik_control!");

    std_msgs::String msg;
    msg.data = "stop";
    left_arm_stop.publish(msg);
    right_arm_stop.publish(msg);
}

stop_window::~stop_window()
{

}