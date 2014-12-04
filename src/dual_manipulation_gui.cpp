#include "dual_manipulation_gui.h"

dual_manipulation_gui::dual_manipulation_gui():
main_layout(Qt::Vertical),visualization_layout(Qt::Horizontal),state_layout(Qt::Horizontal),control_layout(Qt::Horizontal)
{
    visualization_layout.addWidget(&render);
    visualization_layout.addWidget(&camera);
    
    QList<int> list= visualization_layout.sizes();
    list.replace(0,visualization_layout.width()/0.5);
    list.replace(1,visualization_layout.width()/0.5);
    visualization_layout.setSizes(list);
    
    state_layout.addWidget(&state_machine);
    state_layout.addWidget(&state);
    
    QList<int> list2= state_layout.sizes();
    list2.replace(0,state_layout.width()/0.5);
    list2.replace(1,state_layout.width()/0.5);
    state_layout.setSizes(list2);
    
    control_layout.addWidget(&control);
  
    main_layout.addWidget(&visualization_layout);
    main_layout.addWidget(&state_layout);
    main_layout.addWidget(&control_layout);
    
    QList<int> list3= main_layout.sizes();
    list3.replace(0,main_layout.height()/0.10);
    list3.replace(1,main_layout.height()/0.15);
    list3.replace(2,main_layout.height()/0.75);
    main_layout.setSizes(list3);
    
    setLayout(new QGridLayout);
    layout()->addWidget(&main_layout);
}

dual_manipulation_gui::~dual_manipulation_gui()
{

}
