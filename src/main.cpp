#include "dual_manipulation_gui.h"
#include <QApplication>
#include <ros/ros.h>
#include "ros/package.h"
#include <thread>

void thread_body()
{
    while(1)
    {
	ros::spinOnce(); 
	usleep(20000);
    }
}

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "Dual_Manipulation_GUI", ros::init_options::AnonymousName );
    }
    
    QApplication a(argc, argv);
    a.setOrganizationName("CentroEPiaggio");
    a.setOrganizationDomain("centropiaggio.unipi.it");
    a.setApplicationName("Dual_Manipulation_GUI");
    dual_manipulation_gui dmg;
    
    struct sigaction sigint;
    sigint.sa_handler = dual_manipulation_gui::intSignalHandler;
    sigemptyset(&sigint.sa_mask);
    sigint.sa_flags |= SA_RESTART;
    if (sigaction(SIGINT, &sigint, 0) > 0) ROS_WARN_STREAM("Something wrong with unix interrupt signal handling");
    
    QString path_to_package = QString::fromStdString(ros::package::getPath("dual_manipulation_gui"));
    
    dmg.setWindowTitle("Dual Manipulation GUI");
    dmg.setWindowIcon(QIcon(path_to_package + "/vito.png"));
    dmg.show();
    
    std::thread th(&thread_body);

    return a.exec();
}