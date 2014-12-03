#include "dual_manipulation_gui.h"

dual_manipulation_gui::dual_manipulation_gui()
{
    main_layout.addLayout(&visualization_layout);
    main_layout.addLayout(&control_layout);
    main_layout.addLayout(&commands_layout);
    
    setLayout(&main_layout);
}

dual_manipulation_gui::~dual_manipulation_gui()
{

}
