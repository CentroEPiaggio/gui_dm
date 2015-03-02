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
class Viewer : public QGraphicsView
{
public:
    Viewer ( QWidget* parent = 0 );
    ~Viewer();
    void setScalingFactor ( double scalingFactorX, double scalingFactorY );
    void setTranslateFactor ( double tx=0,double ty=0 );
    void setBackImage ( std::string path );

protected:
    virtual void wheelEvent(QWheelEvent* event);

private:
    void closeEvent ( QCloseEvent *event );

    int timerId;

    std::string backImage;
    QImage image;
    QPixmap pixmap;
    QGraphicsScene* Scene;

public:
    double scalingFactorX, scalingFactorY, translateX, translateY;
    void setScalingAndTranslateFactor ( double maxX,double minX,double maxY,double minY );
    double maxX;
    double minX;
    double maxY;
    double minY;
};

#endif //VIEWER
