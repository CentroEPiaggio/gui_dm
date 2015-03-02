#include "widgets/state_machine_widget.h"
#include "ros/package.h"

state_machine_widget::state_machine_widget()
{
    std::string path=ros::package::getPath("dual_manipulation_planner");
    img_path=path+"/image.eps";
    
    label = new QLabel();
    QSizePolicy policy = QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    label->setSizePolicy(policy);
    label->setMinimumSize(30,30);

    label_layout = new QGridLayout();
    label_layout->addWidget(label);

    main_layout.addLayout(label_layout,0,0,Qt::AlignCenter);

    setLayout(&main_layout);

    connect(&timer, SIGNAL(timeout()), this, SLOT(timer_body()));
}

void state_machine_widget::timer_body()
{
    QPixmap pixmap;
    int w = label->width();
    int h = label->height();
    QImage immagine;
    immagine=QImage ( QString ( img_path.c_str() ) );
    immagine=immagine.scaled ( QSize ( width(),height() ), Qt::IgnoreAspectRatio,Qt::SmoothTransformation );
    pixmap.convertFromImage ( immagine );

    label->setPixmap(pixmap);
    label->adjustSize();
}

void state_machine_widget::start_timer()
{
    timer.start(2000);
}

void state_machine_widget::stop_timer()
{
    timer.stop();
}

state_machine_widget::~state_machine_widget()
{

}