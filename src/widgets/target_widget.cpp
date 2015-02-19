#include "widgets/target_widget.h"
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"

target_widget::target_widget()
{
    int glob_id = 0;
    int row = 0;
    int col = 0;
    
    gui_target_service = n.advertiseService("gui_target_service", &target_widget::gui_target_service_callback, this);
    sub = n.subscribe("/clicked_point",1,&target_widget::clicked_point,this);
    
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
    
    sub_im = n.subscribe("/target_marker",1,&target_widget::update_position,this);
    server = new interactive_markers::InteractiveMarkerServer("target_interactive_marker");
    int_marker = new visualization_msgs::InteractiveMarker();
    int_marker->header.frame_id = "/world";
    int_marker->name = "target__interactive";
    int_marker->description = "";
    pub_target = n.advertise<visualization_msgs::Marker>( "/target_marker", 1000 );
    im_sub_fb = n.subscribe("/target_interactive_marker/feedback",1,&target_widget::im_callback,this);
    
    target_marker.color.a=1;
    target_marker.color.r=1;
    target_marker.color.g=0;
    target_marker.color.b=0;
    target_marker.header.frame_id = "/world";
    target_marker.id=1;
    target_marker.ns="target";
    target_marker.lifetime = ros::DURATION_MAX;
    target_marker.scale.x = 0.1;
    target_marker.scale.y = 0.1;
    target_marker.scale.z = 0.1;
    target_marker.type = visualization_msgs::Marker::CYLINDER;
    target_marker.pose.orientation.w=1;
}

void target_widget::im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
{     
    target_pose = feedback.pose;
    update_coords();
}

void target_widget::update_position(const visualization_msgs::Marker &marker_)
{
    int_marker->controls.clear();

    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( marker_);

    int_marker->controls.push_back( box_control );

    visualization_msgs::InteractiveMarkerControl control;
    
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker->controls.push_back(control);
    
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker->controls.push_back(control);
    
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker->controls.push_back(control);
    
    int_marker->pose=marker_.pose;
    int_marker->name=marker_.ns;
    int_marker->scale=0.2;

    server->insert(*int_marker);

    server->applyChanges();

}

void target_widget::clicked_point(const geometry_msgs::PointStampedPtr& point)
{
    target_pose.position = point->point;
    update_coords();
    publish_marker();
}

bool target_widget::gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request& req, dual_manipulation_shared::gui_target_service::Response& res)
{
    res.ack = true;

    ROS_INFO_STREAM("Accepted request to set target pose: "<<req.info.c_str()<<" |> please set the object position <|");
    
    while(!target_ready){ros::spinOnce();};
    
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

    publish_marker();
}

void target_widget::publish_marker()
{
    target_marker.pose = target_pose;
    pub_target.publish(target_marker);
}

void target_widget::update_coords()
{
    coord_map.at(0)->setText(QString::number(target_pose.position.x, 'f', 2));
    coord_map.at(1)->setText(QString::number(target_pose.position.y, 'f', 2));
    coord_map.at(2)->setText(QString::number(target_pose.position.z, 'f', 2));
    
    double ro,pi,ya;
    tf::Quaternion q;
    tf::quaternionMsgToTF(target_pose.orientation,q);
    tf::Matrix3x3(q).getRPY(ro,pi,ya);
    
    coord_map.at(3)->setText(QString::number(ro, 'f', 2));
    coord_map.at(4)->setText(QString::number(pi, 'f', 2));
    coord_map.at(5)->setText(QString::number(ya, 'f', 2));
}

void target_widget::on_set_target_clicked()
{
    ROS_INFO_STREAM("Target Set!");
    
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