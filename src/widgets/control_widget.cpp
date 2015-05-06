#include "widgets/control_widget.h"
#include "ros/ros.h"
#include "ros/package.h"

control_widget::control_widget(state_machine_widget* smw_):smw(smw_)
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

    connect(&stop_robot_button,SIGNAL(clicked(bool)), this, SLOT(on_stop_robot_button_clicked()));

    home_robot_button.setFixedSize(45,45);
    home_robot_button.setIcon(QIcon(path_to_package + "/home.png"));
    home_robot_button.setIconSize( QSize(home_robot_button.size().width(), home_robot_button.size().height() ));

    connect(&home_robot_button,SIGNAL(clicked(bool)), this, SLOT(on_home_robot_button_clicked()));

    main_layout.addWidget(&home_robot_button,row+2,0,Qt::AlignCenter);
    main_layout.addWidget(&stop_robot_button,row+2,2,Qt::AlignCenter);

    setLayout(&main_layout);
    
    client = n.serviceClient<dual_manipulation_shared::state_manager_service>("state_manager_ros_service");
    ik_client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
}

void control_widget::on_home_robot_button_clicked()
{
    ROS_DEBUG_STREAM("command HOME to ik_control");

    ik_srv.request.command = "home";
    ik_srv.request.ee_name = "both_hands";

    if (ik_client.call(ik_srv))
    {
	ROS_INFO_STREAM("IK Control Request \'" << ik_srv.request.command << "\' accepted: (" << (int)srv.response.ack << ")");
    }
    else
    {
	ROS_ERROR_STREAM("Failed to call service dual_manipulation_shared::ik_service \'" << ik_srv.request.command << "\'");
    }
}


void control_widget::on_stop_robot_button_clicked()
{
    ROS_DEBUG_STREAM("command STOP to ik_control");

    ik_srv.request.command = "stop";

    if (ik_client.call(ik_srv))
    {
	ROS_INFO_STREAM("IK Control Request \'" << ik_srv.request.command << "\' accepted: (" << (int)srv.response.ack << ")");
    }
    else
    {
	ROS_ERROR_STREAM("Failed to call service dual_manipulation_shared::ik_service \'" << ik_srv.request.command << "\'");
    }
    
    srv.request.command = "abort_move";
    srv.request.time = 0;

    if (client.call(srv))
    {
	ROS_INFO_STREAM("State Manager Request \'" << srv.request.command << "\' accepted: (" << (int)srv.response.ack << ")");
    }
    else
    {
	ROS_ERROR_STREAM("Failed to call service dual_manipulation_shared::state_manager_service \'" << srv.request.command << "\'");
    }
}

void control_widget::on_command_button_clicked(const int& id)
{
    ROS_DEBUG_STREAM("command "<<map_button.at(id)->text().toStdString());
    
    srv.request.command = map_button.at(id)->text().toStdString();
    srv.request.time = 0;

    if (client.call(srv))
    {
	ROS_INFO_STREAM("State Manager Request \'" << srv.request.command << "\' accepted: (" << (int)srv.response.ack << ")");
    }
    else
    {
	ROS_ERROR_STREAM("Failed to call service dual_manipulation_shared::state_manager_service \'" << srv.request.command << "\'");
    }

    if(map_button.at(id)->text().toStdString() == "get_info") smw->start_timer();
    if(map_button.at(id)->text().toStdString() == "exit") smw->stop_timer();
}

control_widget::~control_widget()
{
    for(auto item:map_button)
    {
        delete item.second;
    }
}