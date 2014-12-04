#include "widgets/render_3d_widget.h"

render_3d_widget::render_3d_widget():visualization_manager(&render_panel)
{
    render_panel.setMinimumSize(30,30);
    render_panel.initialize(visualization_manager.getSceneManager(),&visualization_manager);
    
    visualization_manager.initialize();
    visualization_manager.startUpdate();
    visualization_manager.setFixedFrame("/base_link");
    
    main_layout.addWidget(&render_panel,0,0,Qt::AlignCenter);
    
    setLayout(&main_layout);
}

render_3d_widget::~render_3d_widget()
{

}