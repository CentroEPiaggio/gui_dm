#include "dual_manipulation_gui.h"
#include <dual_manipulation_shared/parsing_utils.h>

dual_manipulation_gui::dual_manipulation_gui(): control(),
main_layout(Qt::Vertical),visualization_layout(Qt::Horizontal),state_layout(Qt::Horizontal),control_layout(Qt::Horizontal)
{
    if (node.getParam("dual_manipulation_parameters", gui_params)) parseParameters(gui_params);

    visualization_layout.addWidget(&graph);
    visualization_layout.addWidget(&state);
    
    QList<int> list= visualization_layout.sizes();
    list.replace(0,visualization_layout.width()/0.5);
    list.replace(1,visualization_layout.width()/0.5);
    visualization_layout.setSizes(list);

    target = new target_widget(setting_source_position);
    
    state_layout.addWidget(target);
    
    control_layout.addWidget(&control);
  

    main_layout.addWidget(&state_layout);
    main_layout.addWidget(&visualization_layout);
    main_layout.addWidget(&control_layout);
    
    QList<int> list3= main_layout.sizes();
    list3.replace(0,main_layout.height()/0.4);
    list3.replace(1,main_layout.height()/0.2);
    list3.replace(2,main_layout.height()/0.4);
    main_layout.setSizes(list3);
    
    setLayout(new QGridLayout);
    layout()->addWidget(&main_layout);
}


void dual_manipulation_gui::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    bool use_vision = !setting_source_position;
    
    parseSingleParameter(params,use_vision,"use_vision");
    setting_source_position = !use_vision;
}

dual_manipulation_gui::~dual_manipulation_gui()
{

}
