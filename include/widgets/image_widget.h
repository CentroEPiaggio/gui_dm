#ifndef VIEWER
#define VIEWER

#include <QtGui/QWidget>
#include <QtGui/QKeyEvent>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QTextStream>
#include <QScrollBar>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QDebug>
#include <mutex>
#include <ros/node_handle.h>
#include <dual_manipulation_shared/graph.h>
class Viewer : public QGraphicsView
{
public:
    Viewer ( QWidget* parent = 0 );
    ~Viewer();
    void setScalingFactor ( double scalingFactorX, double scalingFactorY );
    void setTranslateFactor ( double tx=0,double ty=0 );
    void setBackImage ( std::string path );
    virtual void paintEvent ( QPaintEvent *event );
    
protected:
    virtual void wheelEvent(QWheelEvent* event);
    void closeEvent ( QCloseEvent *event );
    virtual void saveSettings(std::string name);
    virtual void loadSettings(std::string name);
    int timerId;
    std::string backImage;
    QImage image;
    QPixmap pixmap;
    QGraphicsScene* Scene;
    std::mutex mutex;
    bool new_message;
    ros::Subscriber graph_sub;
    ros::NodeHandle node;
    std::string name;
private:
    void subscriber_callback(const dual_manipulation_shared::graph::ConstPtr& graph_msg);
    dual_manipulation_shared::graph graph_msg;


public:
    double scalingFactorX, scalingFactorY, translateX, translateY;
    void setScalingAndTranslateFactor ( double maxX,double minX,double maxY,double minY );
    double maxX;
    double minX;
    double maxY;
    double minY;
    
    void set_ns(std::string ns);
};

#endif //VIEWER
