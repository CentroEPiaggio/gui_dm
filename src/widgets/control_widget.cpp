#include "widgets/control_widget.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <std_msgs/String.h>

control_widget::control_widget(std::vector<std::string> ns_list, message_widget* message_, target_widget* target_):message(message_), target(target_)
{
    QString path_to_package = QString::fromStdString(ros::package::getPath("dual_manipulation_gui"));

    int glob_id = 0;
    int row = 0;
    int col = 0;
    
    std::vector<std::string> command_vec;
    command_vec.push_back("get_info");
    command_vec.push_back("plan");
    command_vec.push_back("abort_plan");
    command_vec.push_back("start_moving");
    command_vec.push_back("abort_move");
    command_vec.push_back("exit");
    command_vec.push_back("reset");

    for(auto item:command_vec)
    {
        QPushButton* button = new QPushButton();
	button->setText(QString::fromStdString(item));
	map_button[glob_id]=button;

	connect (button, SIGNAL(clicked()), &signalMapper, SLOT(map())) ;
	signalMapper.setMapping(button, glob_id) ;
	glob_id++;
	
	if(glob_id == command_vec.size()/2+1 || glob_id == command_vec.size())
	{
	    row++;
	    col=0;
	    if(item=="reset") col=1;
	}
	main_layout.addWidget(button,row,col++,Qt::AlignCenter);
    }
    
    connect (&signalMapper, SIGNAL(mapped(int)), this, SLOT(on_command_button_clicked(int))) ;
    
    stop_robot_button.setFixedSize(45,45);
    stop_robot_button.setIcon(QIcon(path_to_package + "/stop.png"));
    stop_robot_button.setIconSize( QSize(stop_robot_button.size().width(), stop_robot_button.size().height() ));

    start_robot_button.setFixedSize(45,45);
    start_robot_button.setIcon(QIcon(path_to_package + "/start.png"));
    start_robot_button.setIconSize( QSize(start_robot_button.size().width(), start_robot_button.size().height() ));
	
	left_arm_stop = n.advertise<std_msgs::String>("/left_arm/emergency_stop",10);
	right_arm_stop = n.advertise<std_msgs::String>("/right_arm/emergency_stop",10);

    connect(&stop_robot_button,SIGNAL(clicked(bool)), this, SLOT(on_stop_robot_button_clicked()));
    connect(&start_robot_button,SIGNAL(clicked(bool)), this, SLOT(on_start_robot_button_clicked()));

    home_robot_button.setFixedSize(45,45);
    home_robot_button.setIcon(QIcon(path_to_package + "/home.png"));
    home_robot_button.setIconSize( QSize(home_robot_button.size().width(), home_robot_button.size().height() ));

    connect(&home_robot_button,SIGNAL(clicked(bool)), this, SLOT(on_home_robot_button_clicked()));
    
    quick_button.setFixedSize(45,45);
    quick_button.setIcon(QIcon(path_to_package + "/quick.png"));
    quick_button.setIconSize( QSize(quick_button.size().width(), quick_button.size().height() ));

    connect(&quick_button,SIGNAL(clicked(bool)), this, SLOT(on_quick_button_clicked()));


    QHBoxLayout* bt_layout = new QHBoxLayout();
    
    bt_layout->addWidget(&quick_button,Qt::AlignCenter);
    bt_layout->addWidget(&home_robot_button,Qt::AlignCenter);
    bt_layout->addWidget(&start_robot_button,Qt::AlignCenter);
    bt_layout->addWidget(&stop_robot_button,Qt::AlignCenter);

    QVBoxLayout* ex_layout = new QVBoxLayout();
    ex_layout->addLayout(&main_layout);
    ex_layout->addLayout(bt_layout);
    setLayout(ex_layout);
    
    if(ns_list.size()==0)
    {
        ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::state_manager_service>("state_manager_ros_service");
        ros::ServiceClient ik_client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
        clients.push_back(client);
        ik_clients.push_back(ik_client);
    }
    else
    {
        for(auto ns:ns_list)
        {
            ros::ServiceClient client = n.serviceClient<dual_manipulation_shared::state_manager_service>(ns + "/state_manager_ros_service");
            ros::ServiceClient ik_client = n.serviceClient<dual_manipulation_shared::ik_service>(ns + "/ik_ros_service");
            clients.push_back(client);
            ik_clients.push_back(ik_client);
        }
    }
}

void control_widget::on_quick_button_clicked()
{
    target->press_publish_marker();
    usleep(200000);
    on_command_button_clicked(0);
    usleep(200000);
    target->press_set_target();
    usleep(1000000);
    on_command_button_clicked(1);
}

void control_widget::on_home_robot_button_clicked()
{
    ROS_DEBUG_STREAM("command HOME to ik_control");

    ik_srv.request.command = "home";
    ik_srv.request.ee_name = "full_robot";

    for(auto ik_client:ik_clients)
    {
        if (ik_client.call(ik_srv))
        {
            std::string msg = "IK Control Request \'" + ik_client.getService() + "::" + ik_srv.request.command + "\' accepted: (" + std::to_string((int)srv.response.ack) + ")";
            if(message!=NULL)
                message->info_message(msg);
            ROS_INFO_STREAM(msg);
            break;
        }
        else
        {
            std::string msg = "Failed to call service \'" + ik_client.getService() + "::" + ik_srv.request.command + "\'";
            if(message!=NULL)
                message->error_message(msg);
            ROS_ERROR_STREAM(msg);
        }
    }
    static ros::Publisher pub = n.advertise<dual_manipulation_shared::graph>("computed_graph",1);
    dual_manipulation_shared::graph g;
    pub.publish(g);
}


void control_widget::on_stop_robot_button_clicked()
{
    ROS_DEBUG_STREAM("command STOP to ik_control");
	
	std_msgs::String msg;
	msg.data = "stop";
	left_arm_stop.publish(msg);
	right_arm_stop.publish(msg);

    ik_srv.request.command = "stop";

    for(auto ik_client:ik_clients)
    {
        if (ik_client.call(ik_srv))
        {
            std::string msg = "IK Control Request \'" + ik_client.getService() + "::" + ik_srv.request.command + "\' accepted: (" + std::to_string((int)srv.response.ack) + ")";
            if(message!=NULL)
                message->info_message(msg);
            ROS_INFO_STREAM(msg);
            break;
        }
        else
        {
            std::string msg = "Failed to call service \'" + ik_client.getService() + "::" + ik_srv.request.command + "\'";
            if(message!=NULL)
                message->error_message(msg);
            ROS_ERROR_STREAM(msg);
        }
    }
    
    srv.request.command = "abort_move";
    srv.request.time = 0;

    for(auto client:clients)
    {
        if (client.call(srv))
        {
            std::string msg = "State Manager Request \'" + client.getService() + "::" + srv.request.command + "\' accepted: (" + std::to_string((int)srv.response.ack) + ")";
            if(message!=NULL)
                message->info_message(msg);
            ROS_INFO_STREAM(msg);
            // do not break here, but send the abort to all state_manager's
        }
        else
        {
            std::string msg = "Failed to call service \'" + client.getService() + "::" + srv.request.command + "\'";
            if(message!=NULL)
                message->error_message(msg);
            ROS_ERROR_STREAM(msg);
        }
    }
}

void control_widget::on_start_robot_button_clicked()
{
    std_msgs::String msg;
    msg.data = "start";
    left_arm_stop.publish(msg);
    right_arm_stop.publish(msg);
}
void control_widget::on_command_button_clicked(const int& id)
{
    ROS_DEBUG_STREAM("command "<<map_button.at(id)->text().toStdString());
    
    srv.request.command = map_button.at(id)->text().toStdString();
    srv.request.time = 0;

    for(auto client:clients)
    {
        if (client.call(srv))
        {
            std::string msg = "State Manager Request \'" + client.getService() + "::" + srv.request.command + "\' accepted: (" + std::to_string((int)srv.response.ack) + ")";
            if(message!=NULL)
                message->info_message(msg);
            ROS_INFO_STREAM(msg);
            // do not break here, but send the command to all state_manager's
        }
        else
        {
            std::string msg = "Failed to call service \'" + client.getService() + "::" + srv.request.command + "\'";
            if(message!=NULL)
                message->error_message(msg);
            ROS_ERROR_STREAM(msg);
        }
    }
}

control_widget::~control_widget()
{
    for(auto item:map_button)
    {
        delete item.second;
    }
}
