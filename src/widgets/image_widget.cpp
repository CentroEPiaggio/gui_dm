#include "widgets/image_widget.h"
#include <dual_manipulation_shared/graph.h>
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

#define RADIUS 15

void Viewer::subscriber_callback(const dual_manipulation_shared::graph::ConstPtr& graph_msg)
{
    mutex.lock();
    this->graph_msg=*graph_msg;
    new_message=true;
    mutex.unlock();
    return;
}


Viewer::Viewer ( QWidget* parent) :QGraphicsView ( parent )
{
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    //Set-up the scene
    Scene = new QGraphicsScene(this);
    setScene(Scene);
    scale(1,-1);
    graph_sub=node.subscribe<dual_manipulation_shared::graph>("computed_graph",1,&Viewer::subscriber_callback,this);
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
    mutex.unlock();
    new_message=false;
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

void Viewer::paintEvent ( QPaintEvent *event )
{
    mutex.lock();
    if (new_message)
    {
    auto graph_msg=&this->graph_msg;
    new_message=false;
    Scene->clear();
    QPen temp;
    QBrush brush;
    QPen pen_red;
    QBrush brush_red;
    brush_red.setColor(QColor::fromRgb(200,70,70));
    pen_red.setColor(QColor::fromRgb(200,70,70));
    brush.setColor ( QColor ( "lightgreen" ) );
    std::map<int,int> id_to_position;
    std::set<int> path_node_ids;
    for (int i=0;i<graph_msg->path_node_ids.size(); i++)
    {
        path_node_ids.insert(graph_msg->path_node_ids[i]);
    }
    for ( int i=0; i<graph_msg->node_id.size(); i++ )
    {
        id_to_position[graph_msg->node_id[i]]=i;
        auto item=Scene->addText (QString ( "" ).fromStdString ( graph_msg->text[i] ) );
        if (!path_node_ids.count(graph_msg->node_id[i]))
        {
            auto ellipse=Scene->addEllipse(graph_msg->x[i]*2-RADIUS/2.0, graph_msg->y[i]*2-RADIUS/2.0,RADIUS,RADIUS,temp,brush);
            ellipse->setFlag(QGraphicsItem::ItemIsSelectable,true);
        }
        else
        {
            auto ellipse=Scene->addEllipse(graph_msg->x[i]*2-RADIUS/2.0, graph_msg->y[i]*2-RADIUS/2.0,RADIUS,RADIUS,pen_red,brush_red);
            ellipse->setFlag(QGraphicsItem::ItemIsSelectable,true);
            item->setDefaultTextColor(Qt::red);
        }
        QFont temp=item->font();
        temp.setPointSize(10);
        item->setFont(temp);
        item->setPos(graph_msg->x[i]*2-9, graph_msg->y[i]*2+10);
        item->scale(1,-1);
    }
    
    temp.setBrush(QColor("black"));
    for ( int i=0;i<graph_msg->source.size(); i++ )
    {
        Scene->addLine(graph_msg->x[id_to_position[graph_msg->source[i]]]*2, graph_msg->y[id_to_position[graph_msg->source[i]]]*2, 
                       graph_msg->x[id_to_position[graph_msg->target[i]]]*2, graph_msg->y[id_to_position[graph_msg->target[i]]]*2,temp );
    }
    pen_red.setWidth(5);
    for ( int i=0;i<graph_msg->path_node_ids.size()-1; i++ )
    {
        Scene->addLine(graph_msg->x[id_to_position[graph_msg->path_node_ids[i]]]*2, graph_msg->y[id_to_position[graph_msg->path_node_ids[i]]]*2, 
                       graph_msg->x[id_to_position[graph_msg->path_node_ids[i+1]]]*2, graph_msg->y[id_to_position[graph_msg->path_node_ids[i+1]]]*2,pen_red );
    }
    }
    mutex.unlock();    
    QGraphicsView::paintEvent(event);
}


Viewer::~Viewer()
{
}