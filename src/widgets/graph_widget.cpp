#include "widgets/graph_widget.h"
#include "ros/package.h"
#include <widgets/state_machine_widget.h>
#include <QtSvg>
graph_widget::graph_widget()
{
    std::string path=ros::package::getPath("dual_manipulation_planner");

    viewer= new Viewer(this);

    label_layout = new QGridLayout();
    label_layout->addWidget(viewer);
//     state_machine_widget* temp1= new state_machine_widget();
//     
//     label_layout1 = new QGridLayout();
//     label_layout1->addWidget(temp1);
    main_layout.addLayout(label_layout,0,0,Qt::AlignCenter);
//     main_layout.addLayout(label_layout1,0,1,Qt::AlignCenter);
    
    setLayout(&main_layout);

}

void graph_widget::set_ns(std::string ns)
{
    viewer->set_ns(ns);
}

graph_widget::~graph_widget()
{

}