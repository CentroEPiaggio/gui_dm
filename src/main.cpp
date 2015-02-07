#include "dual_manipulation_gui.h"
#include <QApplication>
#include <ros/ros.h>
#include "ros/package.h"

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "Dual_Manipulation_GUI", ros::init_options::AnonymousName );
    }
    
    QApplication a(argc, argv);
    dual_manipulation_gui dmg;
    
    QString path_to_package = QString::fromStdString(ros::package::getPath("dual_manipulation_gui"));
    
    dmg.setWindowTitle("Dual Manipulation GUI");
    dmg.setWindowIcon(QIcon(path_to_package + "/vito.png"));
    dmg.show();

    return a.exec();
}