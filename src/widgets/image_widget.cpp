#include "widgets/image_widget.h"
#include <QtGui/QPainter>
#include <QtGui/QApplication>
#include <math.h>
#include <QtCore/QLocale>
#include <QSettings>
#include <QtGui/QWidget>
#include <Qt/QtSvg>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <map>
#include <boost/config/posix_features.hpp>

using namespace std;


Viewer::Viewer ( QWidget* parent) :QGraphicsView ( parent )
{
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    //Set-up the scene
    Scene = new QGraphicsScene(this);
    setScene(Scene);
    
    /*
     * Use the following set of instruction to see the coordinate system of viewer.
     */
    /*for (double theta=0;theta<M_PI;theta=theta+0.4)
     *    {
     *        auto line=Scene->addLine(0,0,50,0);
     *        line->setRotation(theta*180.0/M_PI);
     *        auto item=Scene->addText(QString("").setNum(theta));
     *        item->setPos(80*cos(theta)-10,80*sin(theta)+10);
     *        item->scale(1,-1);
}
*/

//Set-up the view
//    setSceneRect(-500, -500, 1000, 1000); if this instruction is not called, the sceneRect will include 

//Use ScrollHand Drag Mode to enable Panning
setDragMode(ScrollHandDrag);
}

void Viewer::wheelEvent(QWheelEvent* event)
{
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    // Scale the view / do the zoom
    double scaleFactor = 1.15;
    if(event->delta() > 0) {
        // Zoom in
        scale(scaleFactor, scaleFactor);
    } else {
        // Zooming out
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
    }
    // Don't call superclass handler here
    // as wheel is normally used for moving scrollbars
    // QGraphicsView::wheelEvent(event);
}

void Viewer::setScalingAndTranslateFactor ( double maxX, double minX, double maxY, double minY )
{
    setSceneRect(minX, minY, maxX-minX, maxY-minY);
}

void Viewer::setBackImage ( string path )
{
    this->backImage=path;
    if ( backImage.compare ( "" ) )
    {
        image=QImage ( QString ( backImage.c_str() ) );
    }
    pixmap.convertFromImage (image.scaled ( QSize ( width()*100,height()*100)
                                                     ,Qt::KeepAspectRatioByExpanding));//,Qt::SmoothTransformation ));
    Scene->addPixmap(pixmap);
    Scene->setSceneRect(pixmap.rect());
//     QGraphicsSvgItem* temp=new QGraphicsSvgItem(path.c_str());
//     Scene->addItem(temp);
}

void Viewer::closeEvent(QCloseEvent *event)
{
    QWidget::closeEvent(event);
}


Viewer::~Viewer()
{
}
