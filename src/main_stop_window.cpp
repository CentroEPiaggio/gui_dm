#include "stop_window.h"
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
      ros::init( argc, argv, "stop_window", ros::init_options::AnonymousName );
    }
    
    QApplication a(argc, argv);
    a.setOrganizationName("CentroEPiaggio");
    a.setOrganizationDomain("centropiaggio.unipi.it");
    a.setApplicationName("stop window");
    stop_window sw;
    
    QString path_to_package = QString::fromStdString(ros::package::getPath("dual_manipulation_gui"));
    
    sw.setWindowTitle("Stop Window");
    sw.setWindowIcon(QIcon(path_to_package + "/stop.png"));
    sw.show();
    
    std::thread th(&thread_body);

    return a.exec();
}