#include "widgets/target_widget.h"
#include <dual_manipulation_shared/gui_target_service.h>
#include "tf/tf.h"
#include <ros/package.h>
#include <QSettings>
#include "tf_conversions/tf_kdl.h"
#include <dual_manipulation_shared/gui_target_response.h>
#include "tf/transform_listener.h"
#include "dual_manipulation_shared/grasp_trajectory.h"
#include "dual_manipulation_shared/serialization_utils.h"

#define OBJ_GRASP_FACTOR 1000
#define DEBUG 1

target_widget::target_widget(bool setting_source_position_, std::vector< std::string > ns_list, state_machine_widget* smw_, graph_widget* gw_, message_widget* message_): setting_source_position(setting_source_position_), smw(smw_), gw(gw_), message(message_)
{
    sub = n.subscribe("/clicked_point",1,&target_widget::clicked_point,this);
    good_grasp_subscriber = n.subscribe("/good_grasp_topic",1,&target_widget::good_grasp_callback,this);
    good_grasp_publisher = n.advertise<visualization_msgs::MarkerArray>( "/good_grasp_marker", 1000, true );

    int glob_id = 0;
    int row = 0;
    int col = 0;

    source_label.setText("Source");
    source_label.setStyleSheet("color : blue");

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
    if(!setting_source_position) edit->setEnabled(false);
    
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
    copy_source.setText("Target=Source");
    publish_button.setText("Publish Markers");
    
    connect(&set_target_button, SIGNAL(clicked()), this, SLOT(on_set_target_clicked()));
    connect(&copy_source, SIGNAL(clicked()), this, SLOT(on_copy_source_clicked()));
    connect(&publish_button, SIGNAL(clicked(bool)), this, SLOT(publish_marker()));

    for(auto item:db_mapper.Objects)
    {
	object_selection.addItem(QString::fromStdString(std::get<0>(item.second)));
	object_checked.push_back(false);
    }
    object_selection.setCurrentIndex(0);
    object_check.setCheckState(object_checked.at(object_selection.currentIndex())?Qt::CheckState::Checked:Qt::CheckState::Unchecked);
    
    connect(&object_selection, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_object_changed()));
    connect(&object_check, SIGNAL(clicked(bool)), this, SLOT(on_object_checked()));

    clicking_pose.addItem("CLICK: source");
    clicking_pose.addItem("CLICK: target");
    clicking_pose.setCurrentIndex(0);

    main_layout.addWidget(&object_check,row+1,0,Qt::AlignCenter);
    main_layout.addWidget(&object_selection,row+1,1,Qt::AlignCenter);
    main_layout.addWidget(&set_target_button,row+1,2,Qt::AlignCenter);
    main_layout.addWidget(&publish_button,row+1,3,Qt::AlignCenter);
    main_layout.addWidget(&copy_source,row+1,4,Qt::AlignCenter);
    if(setting_source_position) main_layout.addWidget(&clicking_pose,row+1,5,Qt::AlignCenter);
  
    setLayout(&main_layout);
    
    
    if(ns_list.size()==0)
    {
        ros::Publisher target_pub = n.advertise<dual_manipulation_shared::gui_target_response>( "gui_target_response", 1000 );
        target_pubs.push_back(target_pub);
        ros::ServiceServer gui_target_service = n.advertiseService("gui_target_service", &target_widget::gui_target_service_callback, this);
        gui_target_services.push_back(gui_target_service);
    }
    else
    {
        obj_max=ns_list.size();
        for(auto ns:ns_list)
        {
            namespaces.push_back(ns);
            ros::Publisher target_pub = n.advertise<dual_manipulation_shared::gui_target_response>( ns+"/gui_target_response", 1000 );
            target_pubs.push_back(target_pub);
            ros::ServiceServer gui_target_service = n.advertiseService(ns + "/gui_target_service", &target_widget::gui_target_service_callback, this);
            gui_target_services.push_back(gui_target_service);
        }
    }
    
    target_pose.orientation.w=1;
    target_pose.orientation.x=0;
    target_pose.orientation.y=0;
    target_pose.orientation.z=0;
    source_pose.orientation.w=1;
    source_pose.orientation.x=0;
    source_pose.orientation.y=0;
    source_pose.orientation.z=0;
    
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
    source_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
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
    target_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
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
    
    for(auto item:db_mapper.Objects)
    {
	target_poses.push_back(target_pose);
    	source_poses.push_back(source_pose);
	source_ids.push_back(std::get<0>(item.second));
	obj_ids.push_back(item.first);
    }

    if(obj_max==1)
    {
        object_check.setEnabled(false);
        object_check.setChecked(true);
    }
}

void target_widget::update_mesh_resources()
{
    std::string path;
    for(auto item:db_mapper.Objects)
    {
    std::string db_obj_name(std::get<0>(item.second));
    if((!source_id.empty() && source_id.compare(db_obj_name) == 0) || 
        (source_id.empty() && (object_selection.currentText().toStdString().compare(0,db_obj_name.length(),db_obj_name) == 0)))
    {
        path = std::get<1>(item.second);
        obj_ids.at(object_selection.currentIndex()) = item.first;
        break;
    }
    }
    
    ROS_DEBUG_STREAM("object path: "<<path<<std::endl);

    source_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    source_marker.mesh_resource = path.c_str();
    
    target_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    target_marker.mesh_resource = path.c_str();
}

void target_widget::on_object_checked()
{
    if(obj_checked==obj_max && object_check.isChecked())
    {
        object_check.setChecked(false);
        return;
    }

    if( object_check.isChecked() )
    {
        obj_checked++;
    }
    else
    {
        obj_checked--;
    }

    object_checked.at(object_selection.currentIndex()) = object_check.isChecked();
}

void target_widget::on_object_changed()
{
    if(source_poses.size()>object_selection.currentIndex())
    {
      if(object_ns_map.count(object_selection.currentText().toStdString())) update_topics();
      target_pose = target_poses.at(object_selection.currentIndex());
      source_pose = source_poses.at(object_selection.currentIndex());
      source_id = source_ids.at(object_selection.currentIndex());
      if(obj_max!=1) object_check.setChecked(object_checked.at(object_selection.currentIndex()));
      update_coords(source_coord_map,source_pose);
      update_coords(target_coord_map,target_pose);
    }
//     update_mesh_resources();
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
        
    ros::Duration sleep_time(0.05);
    callback_mutex.unlock();
    sleep_time.sleep();
}

void target_widget::update_position(const visualization_msgs::Marker &marker_)
{
    if(marker_.id != obj_ids.at(object_selection.currentIndex())) return;

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
    if(setting_source_position)
    {
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
    }
    else
    {
    target_pose.position = point->point;
    update_coords(target_coord_map,target_pose);
    publish_marker();
    }
    callback_mutex.unlock();
}

bool target_widget::gui_target_service_callback(dual_manipulation_shared::gui_target_service::Request& req, dual_manipulation_shared::gui_target_service::Response& res)
{
    res.ack = true;

    std::string msg = "Accepted request to set target pose: "+req.info+" |> please set the object position <|";
    if(message!=NULL) message->info_message(msg);
    ROS_INFO_STREAM(msg);

    if(!setting_source_position)
    {
        if(req.source_poses.poses.size()==0)
        {
            std::string msg = "target_widget::gui_target_service_callback : source_poses is empty!";
            if(message!=NULL) message->error_message(msg);
            ROS_ERROR_STREAM(msg);
            res.ack = false;
            return res.ack;
        }

        tf::TransformListener tf_;
        std::string err_msg;
        
        if(!tf_.waitForTransform("/world",req.source_poses.poses.at(0).parent_frame,ros::Time(0),ros::Duration(3),ros::Duration(0.01),&err_msg))
        {
            std::string msg = "target_widget::gui_target_service_callback : TF ERROR: " + err_msg;
            if(message!=NULL) message->error_message(msg);
            ROS_ERROR_STREAM(msg);
            res.ack = false;
            return res.ack;
        }

        tf::StampedTransform T;
        tf_.lookupTransform("/world",req.source_poses.poses.at(0).parent_frame,ros::Time(0),T);

        KDL::Frame Camera_Object, World_Object, World_Camera;
        tf::transformTFToKDL(T,World_Camera);
        geometry_msgs::Pose object_pose;

        source_poses.clear();
        source_ids.clear();
        object_selection.clear();
        int i=0;
        for(auto pose:req.source_poses.poses)
        {
            tf::poseMsgToKDL(pose.pose,Camera_Object);
            World_Object = World_Camera*Camera_Object;
            tf::poseKDLToMsg(World_Object,object_pose);

            source_poses.push_back(object_pose);
            source_ids.push_back(pose.id);
            object_selection.addItem(QString::fromStdString(pose.name));
        }

        object_selection.setCurrentIndex(0);
        object_check.setCheckState(object_checked.at(object_selection.currentIndex())?Qt::CheckState::Checked:Qt::CheckState::Unchecked);
        on_object_changed();

        source_coord_map.at(0)->setText(QString::number(source_pose.position.x, 'f', 2));
        source_coord_map.at(1)->setText(QString::number(source_pose.position.y, 'f', 2));
        source_coord_map.at(2)->setText(QString::number(source_pose.position.z, 'f', 2));
        double roll,pitch,yaw;
        tf::Quaternion q;
        tf::quaternionMsgToTF(source_pose.orientation,q);
        tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
        source_coord_map.at(3)->setText(QString::number(roll, 'f', 2));
        source_coord_map.at(4)->setText(QString::number(pitch, 'f', 2));
        source_coord_map.at(5)->setText(QString::number(yaw, 'f', 2));
    }

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

    callback_mutex.unlock();
    
    source_poses.at(object_selection.currentIndex())=source_pose;

    publish_marker();
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

    callback_mutex.unlock();
    
    target_poses.at(object_selection.currentIndex())=target_pose;
    
    publish_marker();
}

void target_widget::press_publish_marker()
{
    publish_marker();
}

void target_widget::publish_marker()
{
    for(int i=0;i<target_poses.size();i++)
    {
        if(obj_max==1)
        {
            if(source_ids.at(i)!=object_selection.currentText().toStdString()) continue;
        }
        else
            if(!object_checked.at(i)) continue;
	
        target_marker.pose = target_poses.at(i);
	target_marker.id = obj_ids.at(i);
	target_marker.mesh_resource = std::get<1>(db_mapper.Objects.at(obj_ids.at(i)));
	pub_target.publish(target_marker);
	usleep(10000);
    }
    
    for(int i=0;i<source_poses.size();i++)
    {
	if(obj_max==1)
        {
            if(source_ids.at(i)!=object_selection.currentText().toStdString()) continue;
        }
        else
            if(!object_checked.at(i)) continue;

	source_marker.pose = source_poses.at(i);
	source_marker.id = obj_ids.at(i);
	source_marker.mesh_resource = std::get<1>(db_mapper.Objects.at(obj_ids.at(i)));
	pub_target.publish(source_marker);
	usleep(10000);
    }
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

void target_widget::press_set_target()
{
    on_set_target_clicked();
}

void target_widget::on_set_target_clicked()
{
    ROS_INFO_STREAM("Target Set!");
    QSettings settings;
    if(setting_source_position)
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

    dual_manipulation_shared::gui_target_response msg;

    object_ns_map.clear();
    int obj_req=0;
    for(int q=0;q<target_poses.size();q++)
    {
        if(obj_max==1)
        {
            if(object_selection.itemText(q) != object_selection.currentText())
                continue;
        }
        else
        {
            if(!object_checked.at(q))
                continue;
        }

	msg.target_pose = target_poses.at(q);
	msg.source_pose = source_poses.at(q);
	msg.obj_id = obj_ids.at(q);
	msg.name = object_selection.itemText(q).toStdString();

	target_pubs.at(obj_req).publish(msg);
        if(namespaces.size()) object_ns_map[msg.name] = namespaces.at(obj_req);
	obj_req++;
	usleep(1000);
    }

    update_topics();
}

void target_widget::update_topics()
{
    if(!object_ns_map.size()) return;
    smw->set_ns(object_ns_map.at(object_selection.currentText().toStdString()));
    gw->set_ns(object_ns_map.at(object_selection.currentText().toStdString()));
}

void target_widget::on_copy_source_clicked()
{
    target_pose = source_pose;
    update_coords(target_coord_map,target_pose);
    publish_marker();
}

void target_widget::good_grasp_callback(dual_manipulation_shared::good_grasp_msg msg)
{
    dual_manipulation_shared::grasp_trajectory grasp_msg;
    std::string file_name;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker text_marker;
    visualization_msgs::MarkerArray markers;

    marker.action=3; //delete all
    marker.header.frame_id = "world";
    markers.markers.push_back(marker);
    //good_grasp_publisher.publish(marker);

    marker.action=visualization_msgs::Marker::ADD;
    marker.lifetime=ros::DURATION_MAX;
    marker.type=visualization_msgs::Marker::MESH_RESOURCE;
    std::string path_r = "package://soft_hand_description/meshes/palm_right.stl";
    std::string path_l = "package://soft_hand_description/meshes/palm_left.stl";
    marker.scale.x=0.001;
    marker.scale.y=0.001;
    marker.scale.z=0.001;

    text_marker.header = marker.header;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0;
    text_marker.scale.z = 0.1;

    for(int i=0; i < msg.good_source_grasps.size(); i++)
    {
        int grasp_id_ = msg.good_source_grasps.at(i);
        file_name = "object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_ % OBJ_GRASP_FACTOR);
        if(deserialize_ik(grasp_msg,file_name))
        {
            ROS_DEBUG_STREAM("Deserialization object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_) << " OK!");
            if(!db_mapper.Objects.count(obj_ids.at(object_selection.currentIndex())))
            {
                std::string msg = "Object "+std::to_string(obj_ids.at(object_selection.currentIndex()))+" is not in the database! . . . Retry!";
                if(message!=NULL) message->warning_message(msg);
                ROS_WARN_STREAM(msg);
                continue;
            }
            if(!db_mapper.Grasps.count(grasp_id_))
            {
                std::string msg = "Grasp #"+std::to_string(grasp_id_)+" is not in the database! . . . Retry!";
                if(message!=NULL) message->warning_message(msg);
                ROS_WARN_STREAM(msg);
                continue;
            }
        }
        else
        {
            std::string msg = "Error in deserialization object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_) + "! . . . Retry!";
            if(message!=NULL) message->warning_message(msg);
            ROS_WARN_STREAM(msg);
            continue;
        }
    
        endeffector_id ee_id = db_mapper.Grasps.at(grasp_id_).ee_id;
        marker.color.a = 0.75;
        marker.color.b = (ee_id==1)?0:1;
        marker.color.g = (ee_id==1)?1:0;
        marker.color.r = 0;
        marker.mesh_resource=(ee_id==1)?(path_l.c_str()):(path_r.c_str());
        marker.id = grasp_id_;
        marker.ns="source";
        KDL::Frame source_hand,world_source;
        tf::poseMsgToKDL(grasp_msg.ee_pose.back(),source_hand);
        tf::poseMsgToKDL(source_pose,world_source);
        tf::poseKDLToMsg(world_source*source_hand,marker.pose);

#if DEBUG>0
        text_marker.text = std::string( "(" ) + std::to_string( marker.id ) + std::string( "," ) + std::to_string( ee_id ) + std::string ( ")" );
        text_marker.id = marker.id;
        text_marker.ns = "source/text";
        text_marker.pose.position = marker.pose.position;
        text_marker.color = marker.color;
        text_marker.color.a = 1.0;
        markers.markers.push_back(text_marker);
#endif

        markers.markers.push_back(marker);
        //good_grasp_publisher.publish(marker);
    }
    
    for(int i=0; i<msg.good_target_grasps.size(); i++)
    {
        int grasp_id_ = msg.good_target_grasps.at(i);
        file_name = "object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_ % OBJ_GRASP_FACTOR);
        if(deserialize_ik(grasp_msg,file_name))
        {
            ROS_DEBUG_STREAM("Deserialization object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_) + " OK!");
            if(!db_mapper.Objects.count(obj_ids.at(object_selection.currentIndex())))
            {
                std::string msg = "Object "+std::to_string(obj_ids.at(object_selection.currentIndex()))+" is not in the database! . . . Retry!";
                if(message!=NULL) message->warning_message(msg);
                ROS_WARN_STREAM(msg);
                continue;
            }
            if(!db_mapper.Grasps.count(grasp_id_))
            {
                std::string msg = "Grasp #"+std::to_string(grasp_id_)+" is not in the database! . . . Retry!";
                if(message!=NULL) message->warning_message(msg);
                ROS_WARN_STREAM(msg);
                continue;
            }
        }
        else
        {
            std::string msg = "Error in deserialization object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_) + "! . . . Retry!";
            if(message!=NULL) message->warning_message(msg);
            ROS_WARN_STREAM(msg);
            continue;
        }
    
        endeffector_id ee_id = db_mapper.Grasps.at(grasp_id_).ee_id;
        marker.color.a = 0.75;
        marker.color.b = (ee_id==1)?0:1;
        marker.color.g = (ee_id==1)?1:0;
        marker.color.r = 0;
        marker.mesh_resource=(ee_id==1)?(path_l.c_str()):(path_r.c_str());
        marker.id=grasp_id_;
        marker.ns="target";
        KDL::Frame target_hand,world_target;
        tf::poseMsgToKDL(grasp_msg.attObject.object.mesh_poses.front(),target_hand);
        target_hand = target_hand.Inverse();
        tf::poseMsgToKDL(target_pose,world_target);
        tf::poseKDLToMsg(world_target*target_hand,marker.pose);

#if DEBUG>0
        text_marker.text = std::string( "(" ) + std::to_string( marker.id ) + std::string( "," ) + std::to_string( ee_id ) + std::string ( ")" );
        text_marker.id = marker.id;
        text_marker.ns = "target/text";
        text_marker.pose.position = marker.pose.position;
        text_marker.color = marker.color;
        text_marker.color.a = 1.0;
        markers.markers.push_back(text_marker);
#endif

        markers.markers.push_back(marker);
        //good_grasp_publisher.publish(marker);
    }

#if DEBUG>0
    for(int i=0;i<msg.bad_source_grasps.size();i++)
    {
        int grasp_id_ = msg.bad_source_grasps.at(i);
        file_name = "object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_ % OBJ_GRASP_FACTOR);
        if(!deserialize_ik(grasp_msg,file_name))
        {
            std::string msg = "Error in deserialization object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_) + "! . . . Retry!";
            if(message!=NULL) message->warning_message(msg);
            ROS_WARN_STREAM(msg);
            continue;
        }
    
        endeffector_id ee_id = db_mapper.Grasps.at(grasp_id_).ee_id;
        marker.color.a = 0.1;
        marker.color.b = (ee_id==1)?0:0.1;
        marker.color.g = (ee_id==1)?0.1:0;
        marker.color.r = 1;
        marker.mesh_resource=(ee_id==1)?(path_l.c_str()):(path_r.c_str());
        marker.id = grasp_id_;
        marker.ns="bad_source";
        KDL::Frame source_hand,world_source;
        tf::poseMsgToKDL(grasp_msg.ee_pose.back(),source_hand);
        tf::poseMsgToKDL(source_pose,world_source);
        tf::poseKDLToMsg(world_source*source_hand,marker.pose);

        text_marker.text = std::string( "(" ) + std::to_string( marker.id ) + std::string( "," ) + std::to_string( ee_id ) + std::string ( ")" );
        text_marker.id = marker.id;
        text_marker.ns = "bad_source/text";
        text_marker.pose.position = marker.pose.position;
        text_marker.color = marker.color;
        text_marker.color.a = 1.0;

        markers.markers.push_back(text_marker);
        markers.markers.push_back(marker);
        //good_grasp_publisher.publish(marker);
    }
    
    for(int i=0;i<msg.bad_target_grasps.size();i++)
    {
        int grasp_id_ = msg.bad_target_grasps.at(i);
        file_name = "object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_ % OBJ_GRASP_FACTOR);
        if(!deserialize_ik(grasp_msg,file_name))
        {
            std::string msg = "Error in deserialization object" + std::to_string(obj_ids.at(object_selection.currentIndex())) + "/grasp" + std::to_string(grasp_id_) + "! . . . Retry!";
            if(message!=NULL) message->warning_message(msg);
            ROS_WARN_STREAM(msg);
            continue;
        }
    
        endeffector_id ee_id = db_mapper.Grasps.at(grasp_id_).ee_id;
        marker.color.a = 0.1;
        marker.color.b = (ee_id==1)?0:0.1;
        marker.color.g = (ee_id==1)?0.1:0;
        marker.color.r = 1;
        marker.mesh_resource=(ee_id==1)?(path_l.c_str()):(path_r.c_str());
        marker.id = grasp_id_;
        marker.ns="bad_target";
        KDL::Frame target_hand,world_target;
        tf::poseMsgToKDL(grasp_msg.attObject.object.mesh_poses.front(),target_hand);
        target_hand = target_hand.Inverse();
        tf::poseMsgToKDL(target_pose,world_target);
        tf::poseKDLToMsg(world_target*target_hand,marker.pose);

        text_marker.text = std::string( "(" ) + std::to_string( marker.id ) + std::string( "," ) + std::to_string( ee_id ) + std::string ( ")" );
        text_marker.id = marker.id;
        text_marker.ns = "bad_target/text";
        text_marker.pose.position = marker.pose.position;
        text_marker.color = marker.color;
        text_marker.color.a = 1.0;

        markers.markers.push_back(text_marker);
        markers.markers.push_back(marker);
        // good_grasp_publisher.publish(marker);
    }
#endif

    std::string msg__ = "Publishing good grasp markers";
    if(message!=NULL) message->info_message(msg__);
    ROS_INFO_STREAM(msg__);
    good_grasp_publisher.publish(markers);
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