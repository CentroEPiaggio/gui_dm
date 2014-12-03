#include "dual_manipulation_gui.h"

dual_manipulation_gui::dual_manipulation_gui():
main_layout(Qt::Vertical),visualization_layout(Qt::Horizontal),state_layout(Qt::Horizontal),control_layout(Qt::Horizontal)
{
    visualization_layout.addWidget(&render);
    visualization_layout.addWidget(&camera);
    
    state_layout.addWidget(&state_machine);
    state_layout.addWidget(&state);
    
    control_layout.addWidget(&control);
  
    main_layout.addWidget(&visualization_layout);
    main_layout.addWidget(&state_layout);
    main_layout.addWidget(&control_layout);
    
    setLayout(new QGridLayout);
    layout()->addWidget(&main_layout);
}

dual_manipulation_gui::~dual_manipulation_gui()
{

}
