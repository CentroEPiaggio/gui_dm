#include "widgets/control_widget.h"

control_widget::control_widget()
{
    example_button.setText("Example Button");
    
    main_layout.addWidget(&example_button,0,0,Qt::AlignCenter);
    
    setLayout(&main_layout);
}

control_widget::~control_widget()
{

}