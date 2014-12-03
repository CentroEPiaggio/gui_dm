#include "dual_manipulation_gui.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
      ros::init( argc, argv, "pilot_interface", ros::init_options::AnonymousName );
    }
    
    QApplication a(argc, argv);
    dual_manipulation_gui dmg;
    
    dmg.setWindowTitle("Dual Manipulation GUI");

    dmg.show();

    return a.exec();
}