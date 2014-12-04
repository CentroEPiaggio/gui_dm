#include "widgets/camera_widget.h"

camera_widget::camera_widget():visualization_manager(&render_panel)
{
    render_panel.setMinimumSize(30,30);
    render_panel.initialize(visualization_manager.getSceneManager(),&visualization_manager);

    visualization_manager.initialize();
    visualization_manager.startUpdate();
    visualization_manager.setFixedFrame("/base_link");

    camera_display = visualization_manager.createDisplay("rviz/Camera", "camera", true );
    camera_display->subProp("Image Topic")->setValue("/camera/rgb/image_raw");

    main_layout.addWidget(camera_display->getAssociatedWidget(),0,0,Qt::AlignCenter);

    setLayout(&main_layout);
}

camera_widget::~camera_widget()
{

}