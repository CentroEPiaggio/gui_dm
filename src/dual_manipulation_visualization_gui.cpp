#include "dual_manipulation_visualization_gui.h"
#include "widgets/image_widget.h"

dual_manipulation_visualization_gui::dual_manipulation_visualization_gui()
{
    Viewer* temp= new Viewer(this);
    label_layout = new QGridLayout();
    label_layout->addWidget(temp);
    main_layout.addLayout(label_layout,0,0,Qt::AlignCenter);
    setLayout(&main_layout);
}

dual_manipulation_visualization_gui::~dual_manipulation_visualization_gui()
{
}
