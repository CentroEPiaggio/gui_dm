#include "widgets/target_widget.h"
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"

target_widget::target_widget()
{
    int glob_id = 0;
    int row = 0;
    int col = 0;
    
    gui_target_service = n.advertiseService("gui_target_service", &target_widget::gui_target_service_callback, this);
    sub = n.subscribe("/clicked_point ",1,&target_widget::clicked_point,this);
    
    std::vector<std::string> coord_vec;
    coord_vec.push_back("x [m]");
    coord_vec.push_back("y [m]");
    coord_vec.push_back("z [m]");
    coord_vec.push_back("ro [rad]");
    coord_vec.push_back("pi [rad]");
    coord_vec.push_back("ya [rad]");
    
    for(auto item:coord_vec)
    {
	QHBoxLayout* temp_layout = new QHBoxLayout();
	QLabel* label = new QLabel(QString::fromStdString(item));
	label->setFixedSize(60,30);
	coord_label.push_back(label);
	QLineEdit* edit = new QLineEdit();
	edit->setFixedSize(80,30);
	edit->setText(QString::number(0.0, 'f', 2));
	coord_map[glob_id] = edit;
	temp_layout->addWidget(label);
	temp_layout->addWidget(edit);
	coord_sublayout.push_back(temp_layout);
	
	connect (edit, SIGNAL(textChanged(QString)),&signalMapper, SLOT(map())) ;
	signalMapper.setMapping(edit, glob_id) ;
	glob_id++;

	if(glob_id == coord_vec.size()/2+1)
	{
	    row++;
	    col=0;
	}
	main_layout.addLayout(temp_layout,row,col++,Qt::AlignCenter);
    }
  
    connect (&signalMapper, SIGNAL(mapped(int)), this, SLOT(on_coord_edit_changed(int))) ;
    
    set_target_button.setText("Set Target");
    
    connect(&set_target_button, SIGNAL(clicked()), this, SLOT(on_set_target_clicked()));
    
    main_layout.addWidget(&set_target_button,row+1,coord_vec.size()/4,Qt::AlignCenter);
  
    setLayout(&main_layout);
    
    target_pose.orientation.w=1;
}

void target_widget::clicked_point(const geometry_msgs::PointStampedPtr& point)
{
    ROS_INFO("Target position set!");
    target_pose.position = point->point;
}

bool target_widget::gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request& req, dual_manipulation_shared::gui_target_service::Response& res)
{
    res.ack = true;

    ROS_INFO_STREAM("Accepted request to set target pose: "<<req.info.c_str()<<" |> please set the object position <|");
    
    while(!target_ready);
    
    target_ready=false;
    
    res.target_pose = target_pose;

    return res.ack;
}

void target_widget::on_coord_edit_changed(const int& id)
{
    ROS_DEBUG_STREAM("coord changed "<<'('<<id<<"): "<<coord_map.at(id)->text().toStdString());
    
    if(id==0) target_pose.position.x = coord_map.at(id)->text().toDouble();
    if(id==1) target_pose.position.y = coord_map.at(id)->text().toDouble();
    if(id==2) target_pose.position.z = coord_map.at(id)->text().toDouble();
    
    if(id>2)
    {
	double ro,pi,ya;
	tf::Quaternion q;
	tf::quaternionMsgToTF(target_pose.orientation,q);
	tf::Matrix3x3(q).getRPY(ro,pi,ya);
      
	if(id==3) q.setRPY(coord_map.at(id)->text().toDouble(),pi,ya);
	if(id==4) q.setRPY(ro,coord_map.at(id)->text().toDouble(),ya);
	if(id==5) q.setRPY(ro,pi,coord_map.at(id)->text().toDouble());
	
	tf::quaternionTFToMsg(q,target_pose.orientation);
    }
}

void target_widget::on_set_target_clicked()
{
    ROS_INFO_STREAM("Target Set");
    
    target_ready = true;
}

target_widget::~target_widget()
{
    for(auto item:coord_map)
    {
        delete item.second;
    }
    
    for(auto item:coord_sublayout)
    {
	delete item;
    }
}