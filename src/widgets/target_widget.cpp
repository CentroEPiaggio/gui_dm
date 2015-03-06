#include "widgets/target_widget.h"
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"
#include <ros/package.h>
#include <QSettings>

target_widget::target_widget()
{
    gui_target_service = n.advertiseService("gui_target_service", &target_widget::gui_target_service_callback, this);
    sub = n.subscribe("/clicked_point",1,&target_widget::clicked_point,this);

    int glob_id = 0;
    int row = 0;
    int col = 0;

    source_label.setText("Source");
    source_label.setStyleSheet("color : green");

    target_label.setText("Target");
    target_label.setStyleSheet("color : red");
    
    main_layout.addWidget(&source_label,row,0,Qt::AlignCenter);
    main_layout.addWidget(&target_label,row,2,Qt::AlignCenter);
    
    std::vector<std::string> coord_vec;
    coord_vec.push_back("x [m]");
    coord_vec.push_back("y [m]");
    coord_vec.push_back("z [m]");
    coord_vec.push_back("ro [rad]");
    coord_vec.push_back("pi [rad]");
    coord_vec.push_back("ya [rad]");

    row++;
    QSettings settings;
    
    for(auto item:coord_vec)
    {
	QHBoxLayout* temp_layout = new QHBoxLayout();
	QLabel* label = new QLabel(QString::fromStdString(item));
	label->setFixedSize(60,30);
	source_coord_label.push_back(label);
	QLineEdit* edit = new QLineEdit();
	edit->setFixedSize(80,30);
        QString s("source");
        s.append(QString::number(glob_id));
        double value=0.0;
        value=settings.value(s,value).toDouble();
 	edit->setText(QString::number(value, 'f', 2));
	source_coord_map[glob_id] = edit;
	temp_layout->addWidget(label);
	temp_layout->addWidget(edit);
	source_coord_sublayout.push_back(temp_layout);
	
	connect (edit, SIGNAL(textChanged(QString)),&source_signalMapper, SLOT(map())) ;
	source_signalMapper.setMapping(edit, glob_id) ;
	glob_id++;

	if(glob_id == coord_vec.size()/2+1)
	{
	    row=1;
	    col++;
	}
	main_layout.addLayout(temp_layout,row++,col,Qt::AlignCenter);
    }
  
    connect (&source_signalMapper, SIGNAL(mapped(int)), this, SLOT(on_source_coord_edit_changed(int))) ;    

    glob_id = 0;
    row=1;
    col=2;

    for(auto item:coord_vec)
    {
	QHBoxLayout* temp_layout = new QHBoxLayout();
	QLabel* label = new QLabel(QString::fromStdString(item));
	label->setFixedSize(60,30);
	target_coord_label.push_back(label);
	QLineEdit* edit = new QLineEdit();
	edit->setFixedSize(80,30);
        QString s("target");
        s.append(QString::number(glob_id));
        double value=0.0;
        value=settings.value(s,value).toDouble();
        edit->setText(QString::number(value, 'f', 2));
	target_coord_map[glob_id] = edit;
	temp_layout->addWidget(label);
	temp_layout->addWidget(edit);
	target_coord_sublayout.push_back(temp_layout);
	
	connect (edit, SIGNAL(textChanged(QString)),&target_signalMapper, SLOT(map())) ;
	target_signalMapper.setMapping(edit, glob_id) ;
	glob_id++;

	if(glob_id == coord_vec.size()/2+1)
	{
	    row=1;
	    col++;
	}
	main_layout.addLayout(temp_layout,row++,col,Qt::AlignCenter);
    }

    connect (&target_signalMapper, SIGNAL(mapped(int)), this, SLOT(on_target_coord_edit_changed(int))) ;

    set_target_button.setText("Set Target");
    publish_button.setText("Publish Markers");
    
    connect(&set_target_button, SIGNAL(clicked()), this, SLOT(on_set_target_clicked()));
    connect(&publish_button, SIGNAL(clicked(bool)), this, SLOT(publish_marker()));

    for(auto item:db_mapper.Objects)
    object_selection.addItem(QString::fromStdString(std::get<0>(item.second)));
    object_selection.setCurrentIndex(2);
    
    connect(&object_selection, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_object_changed()));

    clicking_pose.addItem("CLICK: source");
    clicking_pose.addItem("CLICK: target");
    clicking_pose.setCurrentIndex(0);

    main_layout.addWidget(&object_selection,row+1,0,Qt::AlignCenter);
    main_layout.addWidget(&set_target_button,row+1,1,Qt::AlignCenter);
    main_layout.addWidget(&publish_button,row+1,2,Qt::AlignCenter);
    main_layout.addWidget(&clicking_pose,row+1,3,Qt::AlignCenter);
  
    setLayout(&main_layout);
    
    target_pose.orientation.w=1;
    source_pose.orientation.w=1;
    
    sub_im = n.subscribe("/objects_marker",2,&target_widget::update_position,this);
    server = new interactive_markers::InteractiveMarkerServer("objects_interactive_marker");
    int_marker = new visualization_msgs::InteractiveMarker();
    int_marker->header.frame_id = "/world";
    int_marker->name = "source_interactive";
    int_marker->description = "";
    int_marker->name="objects_im_marker";
    int_marker->scale=0.2;
    pub_target = n.advertise<visualization_msgs::Marker>( "/objects_marker", 1000 );
    im_sub_fb = n.subscribe("/objects_interactive_marker/feedback",1,&target_widget::im_callback,this);

    source_marker.color.a=1;
    source_marker.color.r=0;
    source_marker.color.g=0;
    source_marker.color.b=1;
    source_marker.header.frame_id = "/world";
    source_marker.id=1;
    source_marker.ns="source";
    source_marker.lifetime = ros::DURATION_MAX;
    source_marker.scale.x = 1;
    source_marker.scale.y = 1;
    source_marker.scale.z = 1;
    source_marker.pose.orientation.w=1;

    target_marker.color.a=1;
    target_marker.color.r=1;
    target_marker.color.g=0;
    target_marker.color.b=0;
    target_marker.header.frame_id = "/world";
    target_marker.id=1;
    target_marker.ns="target";
    target_marker.lifetime = ros::DURATION_MAX;
    target_marker.scale.x = 1;
    target_marker.scale.y = 1;
    target_marker.scale.z = 1;
    target_marker.pose.orientation.w=1;
    
    source_pose.position.x = source_coord_map.at(0)->text().toDouble();
    source_pose.position.y = source_coord_map.at(1)->text().toDouble();
    source_pose.position.z = source_coord_map.at(2)->text().toDouble();
    tf::Quaternion q;
    q.setRPY(source_coord_map.at(3)->text().toDouble(),source_coord_map.at(4)->text().toDouble(),source_coord_map.at(5)->text().toDouble());
    tf::quaternionTFToMsg(q,source_pose.orientation);
    
    target_pose.position.x = target_coord_map.at(0)->text().toDouble();
    target_pose.position.y = target_coord_map.at(1)->text().toDouble();
    target_pose.position.z = target_coord_map.at(2)->text().toDouble();
    q.setRPY(target_coord_map.at(3)->text().toDouble(),target_coord_map.at(4)->text().toDouble(),target_coord_map.at(5)->text().toDouble());
    tf::quaternionTFToMsg(q,target_pose.orientation);

    update_mesh_resources(); //to set the initial object shape
}

void target_widget::update_mesh_resources()
{
    std::string path = "package://dual_manipulation_grasp_db/object_meshes/";
    for(auto item:db_mapper.Objects)
    {
	if(std::get<0>(item.second) == object_selection.currentText().toStdString())
	{
	    path.append(std::get<1>(item.second));
	    obj_id_ = item.first;
	    break;
	}
    }
    
    ROS_DEBUG_STREAM("object path: "<<path<<std::endl);

    source_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    source_marker.mesh_resource = path.c_str();
    
    target_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    target_marker.mesh_resource = path.c_str();
}

void target_widget::on_object_changed()
{
    update_mesh_resources();
    publish_marker();
}

void target_widget::im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
{   
    callback_mutex.lock();
    if(feedback.marker_name=="source")
    {
	source_pose = feedback.pose;
	update_coords(source_coord_map,source_pose);
    }
    if(feedback.marker_name=="target")
    {
	target_pose = feedback.pose;
	update_coords(target_coord_map,target_pose);
    }
    callback_mutex.unlock();
}

void target_widget::update_position(const visualization_msgs::Marker &marker_)
{
    callback_mutex.lock();
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
    callback_mutex.unlock();
}

void target_widget::clicked_point(const geometry_msgs::PointStampedPtr& point)
{
    callback_mutex.lock();
    if(clicking_pose.currentText().toStdString()=="CLICK: target")
    {
	target_pose.position = point->point;
	update_coords(target_coord_map,target_pose);
	publish_marker();
    }
    if(clicking_pose.currentText().toStdString()=="CLICK: source")
    {
	source_pose.position = point->point;
	update_coords(source_coord_map,source_pose);
	publish_marker();
    }
    callback_mutex.unlock();
}

bool target_widget::gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request& req, dual_manipulation_shared::gui_target_service::Response& res)
{
    res.ack = true;

    ROS_INFO_STREAM("Accepted request to set target pose: "<<req.info.c_str()<<" |> please set the object position <|");
    ros::spinOnce();
    if (!target_ready) res.ack=false;
    target_ready=false;
    res.target_pose = target_pose;
    res.source_pose = source_pose;
    res.obj_id = obj_id_;
    return res.ack;
}

void target_widget::on_source_coord_edit_changed(const int& id)
{
    callback_mutex.lock();
    ROS_DEBUG_STREAM("source coord changed "<<'('<<id<<"): "<<source_coord_map.at(id)->text().toStdString());
    
    if(id==0) source_pose.position.x = source_coord_map.at(id)->text().toDouble();
    if(id==1) source_pose.position.y = source_coord_map.at(id)->text().toDouble();
    if(id==2) source_pose.position.z = source_coord_map.at(id)->text().toDouble();
    
    if(id>2)
    {
	double ro,pi,ya;
	tf::Quaternion q;
	tf::quaternionMsgToTF(source_pose.orientation,q);
	tf::Matrix3x3(q).getRPY(ro,pi,ya);
      
	if(id==3) q.setRPY(source_coord_map.at(id)->text().toDouble(),pi,ya);
	if(id==4) q.setRPY(ro,source_coord_map.at(id)->text().toDouble(),ya);
	if(id==5) q.setRPY(ro,pi,source_coord_map.at(id)->text().toDouble());
	
	tf::quaternionTFToMsg(q,source_pose.orientation);
    }

    publish_marker();
    callback_mutex.unlock();
}

void target_widget::on_target_coord_edit_changed(const int& id)
{
    callback_mutex.lock();
    ROS_DEBUG_STREAM("target coord changed "<<'('<<id<<"): "<<target_coord_map.at(id)->text().toStdString());
    
    if(id==0) target_pose.position.x = target_coord_map.at(id)->text().toDouble();
    if(id==1) target_pose.position.y = target_coord_map.at(id)->text().toDouble();
    if(id==2) target_pose.position.z = target_coord_map.at(id)->text().toDouble();
    
    if(id>2)
    {
	double ro,pi,ya;
	tf::Quaternion q;
	tf::quaternionMsgToTF(target_pose.orientation,q);
	tf::Matrix3x3(q).getRPY(ro,pi,ya);
      
	if(id==3) q.setRPY(target_coord_map.at(id)->text().toDouble(),pi,ya);
	if(id==4) q.setRPY(ro,target_coord_map.at(id)->text().toDouble(),ya);
	if(id==5) q.setRPY(ro,pi,target_coord_map.at(id)->text().toDouble());
	
	tf::quaternionTFToMsg(q,target_pose.orientation);
    }

    publish_marker();
    callback_mutex.unlock();
}

void target_widget::publish_marker()
{
    target_marker.pose = target_pose;
    pub_target.publish(target_marker);
    
    source_marker.pose = source_pose;
    pub_target.publish(source_marker);
}

void target_widget::update_coords(std::map<int,QLineEdit*> coord_map, geometry_msgs::Pose pose)
{
    coord_map.at(0)->setText(QString::number(pose.position.x, 'f', 2));
    coord_map.at(1)->setText(QString::number(pose.position.y, 'f', 2));
    coord_map.at(2)->setText(QString::number(pose.position.z, 'f', 2));
    
    double ro,pi,ya;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation,q);
    tf::Matrix3x3(q).getRPY(ro,pi,ya);
    
    coord_map.at(3)->setText(QString::number(ro, 'f', 2));
    coord_map.at(4)->setText(QString::number(pi, 'f', 2));
    coord_map.at(5)->setText(QString::number(ya, 'f', 2));
}

void target_widget::on_set_target_clicked()
{
    ROS_INFO_STREAM("Target Set!");
    QSettings settings;
    for (auto coord:source_coord_map)
    {
        QString s("source");
        double value=coord.second->text().toDouble();
        s.append(QString::number(coord.first));
        settings.setValue(s,value);
    }
    for (auto coord:target_coord_map)
    {
        QString s("target");
        double value=coord.second->text().toDouble();
        s.append(QString::number(coord.first));
        settings.setValue(s,value);
    }
    target_ready = true;
}

target_widget::~target_widget()
{
    for(auto item:source_coord_map)
    {
        delete item.second;
    }

    for(auto item:target_coord_map)
    {
        delete item.second;
    }

    for(auto item:source_coord_sublayout)
    {
	delete item;
    }

    for(auto item:target_coord_sublayout)
    {
	delete item;
    }
}