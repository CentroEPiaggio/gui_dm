#include "dual_manipulation_visualization_gui.h"
#include "widgets/image_widget.h"
#include <widgets/state_machine_widget.h>

dual_manipulation_visualization_gui::dual_manipulation_visualization_gui()
{
    Viewer* temp= new Viewer(this);
    label_layout = new QGridLayout();
    label_layout->addWidget(temp);
    state_machine_widget* temp1= new state_machine_widget();
    
    label_layout1 = new QGridLayout();
    label_layout1->addWidget(temp1);
    main_layout.addLayout(label_layout,0,0,Qt::AlignCenter);
    main_layout.addLayout(label_layout1,0,1,Qt::AlignCenter);
    
    setLayout(&main_layout);
}

dual_manipulation_visualization_gui::~dual_manipulation_visualization_gui()
{
}
