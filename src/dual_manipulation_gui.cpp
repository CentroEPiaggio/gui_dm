#include "dual_manipulation_gui.h"

dual_manipulation_gui::dual_manipulation_gui()
{
    visualization_layout.addWidget(&render);
    visualization_layout.addWidget(&camera);
    
    state_layout.addWidget(&state_machine);
    state_layout.addWidget(&state);
    
    control_layout.addWidget(&control);
  
    main_layout.addLayout(&visualization_layout);
    main_layout.addLayout(&state_layout);
    main_layout.addLayout(&control_layout);
    
    setLayout(&main_layout);
}

dual_manipulation_gui::~dual_manipulation_gui()
{

}
