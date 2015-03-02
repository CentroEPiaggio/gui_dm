#include "widgets/control_widget.h"
#include "ros/ros.h"
#include "ros/package.h"

control_widget::control_widget()
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

    main_layout.addWidget(&stop_robot_button,row+2,1,Qt::AlignCenter);

    setLayout(&main_layout);
    
    client = n.serviceClient<dual_manipulation_shared::state_manager_service>("state_manager_ros_service");
    ik_client = n.serviceClient<dual_manipulation_shared::ik_service>("ik_ros_service");
}

void control_widget::on_stop_robot_button_clicked()
{
    ROS_DEBUG_STREAM("command STOP to ik_control");

    ik_srv.request.command = "stop";

    if (ik_client.call(ik_srv))
    {
	ROS_INFO_STREAM("IK Control Request accepted: (" << (int)srv.response.ack << ")");
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::ik_service");
    }
}

void control_widget::on_command_button_clicked(const int& id)
{
    ROS_DEBUG_STREAM("command "<<map_button.at(id)->text().toStdString());
    
    srv.request.command = map_button.at(id)->text().toStdString();
    srv.request.time = 0;

    if (client.call(srv))
    {
	ROS_INFO_STREAM("State Manager Request accepted: (" << (int)srv.response.ack << ")");
    }
    else
    {
	ROS_ERROR("Failed to call service dual_manipulation_shared::state_manager_service");
    }
}

control_widget::~control_widget()
{
    for(auto item:map_button)
    {
        delete item.second;
    }
}