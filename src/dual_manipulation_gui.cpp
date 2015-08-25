#include "dual_manipulation_gui.h"
#include <dual_manipulation_shared/parsing_utils.h>
#include <QSettings>
#include <sys/types.h>
#include <sys/socket.h>

int dual_manipulation_gui::sigintFd[2];

dual_manipulation_gui::dual_manipulation_gui(): control(),
main_layout(Qt::Vertical),visualization_layout(Qt::Horizontal),state_layout(Qt::Horizontal),control_layout(Qt::Horizontal)
{
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigintFd)) ROS_WARN_STREAM("Couldn't create SIGINT socketpair");
    snInt = new QSocketNotifier(sigintFd[1],QSocketNotifier::Read,this);
    connect(snInt, SIGNAL(activated(int)), this, SLOT(handleSigInt()));
    
    if (node.getParam("dual_manipulation_parameters", gui_params)) parseParameters(gui_params);

    visualization_layout.addWidget(&graph);
    visualization_layout.addWidget(&state);
    
    QList<int> list= visualization_layout.sizes();
    list.replace(0,visualization_layout.width()/0.5);
    list.replace(1,visualization_layout.width()/0.5);
    visualization_layout.setSizes(list);

    target = new target_widget(setting_source_position);
    
    state_layout.addWidget(target);
    
    control_layout.addWidget(&control);
  

    main_layout.addWidget(&state_layout);
    main_layout.addWidget(&visualization_layout);
    main_layout.addWidget(&control_layout);
    
    QList<int> list3= main_layout.sizes();
    list3.replace(0,main_layout.height()/0.4);
    list3.replace(1,main_layout.height()/0.2);
    list3.replace(2,main_layout.height()/0.4);
    main_layout.setSizes(list3);
    
    setLayout(new QGridLayout);
    layout()->addWidget(&main_layout);
    readSettings();

    std::string b="\033[0;34m";
    std::string n="\033[0m";
    ROS_INFO_STREAM(b<<"Dual Manipulation GUI ready to be used!"<<n);
}

void dual_manipulation_gui::intSignalHandler(int)
{
    char a = 1;
    ::write(sigintFd[0], &a, sizeof(a));
}

void dual_manipulation_gui::handleSigInt()
{
    snInt->setEnabled(false);
    char tmp;
    ::read(sigintFd[1], &tmp, sizeof(tmp));

    std::string b="\033[0;34m";
    std::string n="\033[0m";
    ROS_INFO_STREAM(b<<"CTRL+C intercepted!"<<n);

    snInt->setEnabled(true);
    
    delete this;
}

void dual_manipulation_gui::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    bool use_vision = !setting_source_position;
    
    parseSingleParameter(params,use_vision,"use_vision");
    setting_source_position = !use_vision;
}

dual_manipulation_gui::~dual_manipulation_gui()
{
    std::string b="\033[0;34m";
    std::string n="\033[0m";
    ROS_INFO_STREAM(b<<"Dying..."<<n);
}

void dual_manipulation_gui::closeEvent(QCloseEvent *event)
{
    writeSettings();
}

void dual_manipulation_gui::readSettings()
{
    QSettings qsettings;
    qsettings.beginGroup( "mainwindow" );
    
    restoreGeometry(qsettings.value( "geometry", saveGeometry() ).toByteArray());
    window()->windowState();
//     restoreState(qsettings.value( "savestate", saveState() ).toByteArray());
    move(qsettings.value( "pos", pos() ).toPoint());
    resize(qsettings.value( "size", size() ).toSize());
    if ( qsettings.value( "maximized", window()->isMaximized() ).toBool() )
        showMaximized();
    
    qsettings.endGroup();
}

void dual_manipulation_gui::writeSettings()
{
    QSettings qsettings;
    qsettings.beginGroup( "mainwindow" );
    
    qsettings.setValue( "geometry", saveGeometry() );
//     qsettings.setValue( "savestate", saveState() );
    qsettings.setValue( "maximized", isMaximized() );
    if ( !isMaximized() ) {
        qsettings.setValue( "pos", pos() );
        qsettings.setValue( "size", size() );
    }
    
    qsettings.endGroup();
}