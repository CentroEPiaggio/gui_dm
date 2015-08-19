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

using namespace std;

#define RADIUS 15

void Viewer::subscriber_callback(const dual_manipulation_shared::graph::ConstPtr& graph_msg)
{
    mutex.lock();
    this->graph_msg.node_id=graph_msg->node_id;
    this->graph_msg.path_node_ids=graph_msg->path_node_ids;
    this->graph_msg.source=graph_msg->source;
    this->graph_msg.target=graph_msg->target;
    this->graph_msg.text=graph_msg->text;
    this->graph_msg.x=graph_msg->x;
    this->graph_msg.y=graph_msg->y;
    this->graph_msg.filtered_source=graph_msg->filtered_source;
    this->graph_msg.filtered_target=graph_msg->filtered_target;
    new_message=true;
    mutex.unlock();
    Scene->invalidate();
    return;
}


Viewer::Viewer ( QWidget* parent) :QGraphicsView ( parent ),name("graph")
{
    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
    //Set-up the scene
    Scene = new QGraphicsScene(this);
    setScene(Scene);
    scale(1,-1);
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
    new_message=false;
    mutex.unlock();
    mutex.lock();
    graph_sub=node.subscribe<dual_manipulation_shared::graph>("computed_graph",100,&Viewer::subscriber_callback,this);
    mutex.unlock();
    loadSettings(name);
}

void Viewer::wheelEvent(QWheelEvent* event)
{
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    // Scale the view / do the zoom
    double scaleFactor = 1.15;
    if(event->delta() < 0) {
        // Zoom in
        scale(scaleFactor, scaleFactor);
    } else {
        // Zooming out
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
    }
    saveSettings(name);
    
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

void Viewer::loadSettings(std::string name)
{
    QSettings settings;
    qreal m11,m12,m13,m21,m22,m23,m31,m32,m33;
    if (!settings.contains(QString::fromStdString(name)+"m11"))
        return;
    #define save_m(x) x = settings.value(QString::fromStdString(name)+#x,QGraphicsView::transform().x()).toReal();
//     m11 = settings.value(QString::fromStdString(name)+"m11",QGraphicsView::transform().m11()).toReal();
    save_m(m11)
    save_m(m12)
    save_m(m13)
    save_m(m22)
    save_m(m21)
    save_m(m23)
    save_m(m31)
    save_m(m32)
    save_m(m33)
    #undef save_m
    QTransform q(m11,m12,m13,m21,m22,m23,m31,m32,m33);
    QGraphicsView::setTransform(q);
}

void Viewer::saveSettings(std::string name)
{
    QSettings settings;
    QTransform q = QGraphicsView::transform();
    #define save_m(x)     settings.setValue(QString::fromStdString(name)+#x,q.x());
//     settings.setValue(QString::fromStdString(name)+"m11",q.m11());
    save_m(m11)
    save_m(m12)
    save_m(m13)
    save_m(m22)
    save_m(m21)
    save_m(m23)
    save_m(m31)
    save_m(m32)
    save_m(m33)
    #undef save_m
}

void Viewer::closeEvent(QCloseEvent *event)
{
    saveSettings(name);
    QWidget::closeEvent(event);
}

void Viewer::paintEvent ( QPaintEvent *event )
{
    mutex.lock();
    if (new_message)
    {
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
    for (int i=0;i<graph_msg.path_node_ids.size(); i++)
    {
        path_node_ids.insert(graph_msg.path_node_ids[i]);
    }
    for ( int i=0; i<graph_msg.node_id.size(); i++ )
    {
        id_to_position[graph_msg.node_id[i]]=i;
        auto item=Scene->addText (QString ( "" ).fromStdString ( graph_msg.text[i] ) );
        if (!path_node_ids.count(graph_msg.node_id[i]))
        {
            auto ellipse=Scene->addEllipse(graph_msg.x[i]*2-RADIUS/2.0, graph_msg.y[i]*2-RADIUS/2.0,RADIUS,RADIUS,temp,brush);
            ellipse->setFlag(QGraphicsItem::ItemIsSelectable,true);
        }
        else
        {
            auto ellipse=Scene->addEllipse(graph_msg.x[i]*2-RADIUS/2.0, graph_msg.y[i]*2-RADIUS/2.0,RADIUS,RADIUS,pen_red,brush_red);
            ellipse->setFlag(QGraphicsItem::ItemIsSelectable,true);
            item->setDefaultTextColor(Qt::red);
        }
        QFont temp=item->font();
        temp.setPointSize(10);
        item->setFont(temp);
        item->setPos(graph_msg.x[i]*2-9, graph_msg.y[i]*2+10);
        item->scale(1,-1);
    }
    
    temp.setBrush(QColor("black"));
    for ( int i=0;i<graph_msg.source.size(); i++ )
    {
        Scene->addLine(graph_msg.x[id_to_position[graph_msg.source[i]]]*2, graph_msg.y[id_to_position[graph_msg.source[i]]]*2, 
                       graph_msg.x[id_to_position[graph_msg.target[i]]]*2, graph_msg.y[id_to_position[graph_msg.target[i]]]*2,temp );
    }
    temp.setBrush(QColor("gray"));
    for ( int i=0;i<graph_msg.filtered_source.size(); i++ )
    {
        Scene->addLine(graph_msg.x[id_to_position[graph_msg.filtered_source[i]]]*2, graph_msg.y[id_to_position[graph_msg.filtered_source[i]]]*2, 
                       graph_msg.x[id_to_position[graph_msg.filtered_target[i]]]*2, graph_msg.y[id_to_position[graph_msg.filtered_target[i]]]*2,temp );
    }
    pen_red.setWidth(15);
    for ( int i=1;i<graph_msg.path_node_ids.size(); i++ )
    {
        Scene->addLine(graph_msg.x[id_to_position[graph_msg.path_node_ids[i-1]]]*2, graph_msg.y[id_to_position[graph_msg.path_node_ids[i-1]]]*2, 
                       graph_msg.x[id_to_position[graph_msg.path_node_ids[i]]]*2, graph_msg.y[id_to_position[graph_msg.path_node_ids[i]]]*2,pen_red );
    }
    }
    mutex.unlock();
    QGraphicsView::paintEvent(event);
}


Viewer::~Viewer()
{
    saveSettings(name);
}
