#include "widgets/control_widget.h"
#include "ros/ros.h"

control_widget::control_widget()
{
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

    for(auto item:command_vec)
    {
        QPushButton* button = new QPushButton();
	button->setText(QString::fromStdString(item));
	map_button[glob_id]=button;

	connect (button, SIGNAL(clicked()), &signalMapper, SLOT(map())) ;
	signalMapper.setMapping(button, glob_id) ;
	glob_id++;
	
	if(glob_id == command_vec.size()/2+1)
	{
	    row++;
	    col=0;
	}
	main_layout.addWidget(button,row,col++,Qt::AlignCenter);
    }
    
    connect (&signalMapper, SIGNAL(mapped(int)), this, SLOT(on_command_button_clicked(int))) ;
    
    setLayout(&main_layout);
    
    client = n.serviceClient<dual_manipulation_shared::state_manager_service>("state_manager_ros_service");
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