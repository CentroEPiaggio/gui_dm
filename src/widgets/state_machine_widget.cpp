#include "widgets/state_machine_widget.h"

state_machine_widget::state_machine_widget()
{
    example_button.setText("Example Button");
    
    main_layout.addWidget(&example_button,0,0,Qt::AlignCenter);
    
    setLayout(&main_layout);
}

state_machine_widget::~state_machine_widget()
{

}