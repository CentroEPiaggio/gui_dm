#include "widgets/render_3d_widget.h"

render_3d_widget::render_3d_widget():visualization_manager(&render_panel)
{
    render_panel.setMinimumSize(30,30);
    render_panel.initialize(visualization_manager.getSceneManager(),&visualization_manager);
    
    visualization_manager.initialize();
    visualization_manager.startUpdate();
    visualization_manager.setFixedFrame("/base_link");
    
    robot_display = visualization_manager.createDisplay( "rviz/RobotModel", "Environment", true );;

    object_display = visualization_manager.createDisplay("rviz/Marker","object",true);
    object_display->subProp("Marker Topic")->setValue("/object");

    grasp_display = visualization_manager.createDisplay("rviz/Marker","grasp",true);
    grasp_display->subProp("Marker Topic")->setValue("/grasp");

    main_layout.addWidget(&render_panel,0,0,Qt::AlignCenter);
    
    setLayout(&main_layout);
}

render_3d_widget::~render_3d_widget()
{

}