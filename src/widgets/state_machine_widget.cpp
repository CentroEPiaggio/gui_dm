#include "widgets/state_machine_widget.h"
#include "ros/package.h"
#include <std_msgs/String.h>
#include "widgets/image_widget.h"
#include <QtCore/QSettings>
#define STR_DECLARE(x) const std::string x = #x;

namespace transition
{
    const std::string started = "started";
    STR_DECLARE(get_info);
    STR_DECLARE(got_info);
    STR_DECLARE(plan);
    STR_DECLARE(failed_plan);
    STR_DECLARE(good_plan);
    STR_DECLARE(abort_plan);
    STR_DECLARE(start_moving);
    STR_DECLARE(task_accomplished);
    STR_DECLARE(abort_move);
    STR_DECLARE(exit);
    STR_DECLARE(failed);
}
namespace ik_transition
{
    STR_DECLARE(plan);
    STR_DECLARE(move);
    STR_DECLARE(check_grasp);
    STR_DECLARE(done);
    STR_DECLARE(check_done);
    STR_DECLARE(soft_fail);
    STR_DECLARE(fail);
    STR_DECLARE(need_replan);
}

STR_DECLARE(starting);
STR_DECLARE(steady);
STR_DECLARE(getting_info);
STR_DECLARE(ready);
STR_DECLARE(planning);
STR_DECLARE(planned);
STR_DECLARE(moving);
STR_DECLARE(exiting);
STR_DECLARE(waiting);
STR_DECLARE(ik_planning);
STR_DECLARE(ik_moving);
STR_DECLARE(ik_checking_grasp);
STR_DECLARE(ik_need_replan);
STR_DECLARE(failing);


void state_machine_widget::stateCallback(const std_msgs::String::ConstPtr & msg)
{
    states.at(current_state)->setBrush(QBrush(Qt::white));
    current_state = msg->data;
    if (states.count(current_state))
        states.at(current_state)->setBrush(QBrush(Qt::cyan));
}

state_machine_widget::state_machine_widget():Viewer()
{
    std::string path=ros::package::getPath("dual_manipulation_planner");
    std::vector<std::tuple<std::string,std::pair<std::string,bool>,std::string>> transition_table{
        //------initial state---------+--------- command -----------------------------------+-- final state---- +
        std::make_tuple( starting     , std::make_pair(transition::started,true)            ,    steady         ),
        std::make_tuple( steady       , std::make_pair(transition::get_info,true)           ,    getting_info   ),
        std::make_tuple( getting_info , std::make_pair(transition::got_info,true)           ,    ready          ),
        std::make_tuple( ready        , std::make_pair(transition::plan,true)               ,    planning       ),
        std::make_tuple( ready        , std::make_pair(transition::get_info,true)           ,    getting_info   ),
        std::make_tuple( planning     , std::make_pair(transition::failed_plan,true)        ,    steady         ),
        std::make_tuple( planning     , std::make_pair(transition::good_plan,true)          ,    planned        ),
        std::make_tuple( planned      , std::make_pair(transition::abort_plan,true)         ,    steady         ),
        std::make_tuple( planned      , std::make_pair(transition::start_moving,true)       ,    moving         ),
        std::make_tuple( moving       , std::make_pair(transition::task_accomplished,true)  ,    steady         ),
        std::make_tuple( moving       , std::make_pair(transition::abort_move,true)         ,    steady         ),
        //----------------------------+-----------------------------------------------------+-------------------+
        std::make_tuple( getting_info , std::make_pair(transition::failed,true)             ,    steady            ),

        //------initial state---------------+--------- command ---------------------------------------+-- final state------ +
        std::make_tuple( waiting            , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_planning        , std::make_pair(ik_transition::move,true)                ,   ik_moving         ),
//         std::make_tuple( ik_planning        , std::make_pair(ik_transition::check_grasp,true)         ,   ik_checking_grasp ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::done,true)                ,   exiting           ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
//         std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::check_done,true)          ,   ik_moving         ),
//         std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::soft_fail,true)           ,   ik_moving         ),
//         std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::plan,true)                ,   ik_planning       ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
        std::make_tuple( ik_need_replan     , std::make_pair(ik_transition::need_replan,true)         ,   exiting           ),
        //----------------------------------+---------------------------------------------------------+-------------------- +
//         std::make_tuple( ik_checking_grasp  , std::make_pair(ik_transition::fail,true)                ,   failing           ),
        std::make_tuple( ik_moving          , std::make_pair(ik_transition::fail,true)                ,   failing           ),
        std::make_tuple( ik_planning        , std::make_pair(ik_transition::fail,true)                ,   ik_need_replan    ),
        std::make_tuple( ik_need_replan     , std::make_pair(ik_transition::fail,true)                ,   failing           )
    };
    timer.setInterval(1000);
    connect(&timer,SIGNAL(timeout()),this,SLOT(save()));
    timer.setSingleShot(false);
    QSettings settings;
    
    //pushing a fake transition just to initialize ik failing state
    transition_table.push_back(
        std::make_tuple( failing     , std::make_pair(ik_transition::fail,true)                ,   failing           )
    );
    //pushing a fake transition just to initialize exiting state
    transition_table.push_back(
        std::make_tuple( exiting     , std::make_pair(ik_transition::fail,true)                ,   exiting           )
    );
    for (auto t:transition_table)
    {
        if (!states.count(std::get<0>(t)))
        {
            QPointF coords(30*states.size(),30*states.size());
            if (settings.contains(std::get<0>(t).c_str()))
                coords=settings.value(std::get<0>(t).c_str()).toPointF();
            auto e = Scene->addEllipse(0,0,100,50);
            e->setPos(coords);
            e->setBrush(QBrush(Qt::white));
            e->setFlag(QGraphicsItem::ItemIsSelectable,true);
            e->setFlag(QGraphicsItem::ItemIsMovable,true);
            e->setZValue(5);
            auto s = Scene->addText(QString::fromStdString(std::get<0>(t)));
            s->setPos(coords+QPointF(50-s->boundingRect().width()/2,25+s->boundingRect().height()/2));
            s->scale(1,-1);
            s->setFlag(QGraphicsItem::ItemIsSelectable,true);
            s->setFlag(QGraphicsItem::ItemIsMovable,true);
            s->setZValue(10);
            QList<QGraphicsItem*> l;
            l.append(e);
            l.append(s);
//             auto g = Scene->createItemGroup(l);
//             g->setFlag(QGraphicsItem::ItemIsSelectable,true);
//             g->setFlag(QGraphicsItem::ItemIsMovable,true);
//             g->setPos(coords);
            states[std::get<0>(t)]=e;
        }
    }
    current_state="steady";
    //Removing fake transition
    transition_table.pop_back();
    transition_table.pop_back();
    
    for (auto t:transition_table)
    {
        QLineF main_line(states.at(std::get<0>(t))->scenePos()+QPointF(50,25),states.at(std::get<2>(t))->scenePos()+QPointF(50,25));
        double dx = main_line.normalVector().unitVector().dx();
        double dy = main_line.normalVector().unitVector().dy();
        
        int factor = main_line.length()/3;
        double final_intersect_x=0, final_intersect_y=0;
        final_intersect_x = 50*25*dx/::sqrt(50*50*dy*dy+25*25*dx*dx);
        final_intersect_y = 50*25*dy/::sqrt(50*50*dy*dy+25*25*dx*dx);
        paintArrow(states.at(std::get<0>(t))->scenePos()+QPointF(50,25),states.at(std::get<2>(t))->scenePos()+QPointF(50+final_intersect_x,25+final_intersect_y));
    }
    timer.start();
    
    graph_sub = node.subscribe<std_msgs::String>("state_machine_change",5,&state_machine_widget::stateCallback,this);
    
}

void state_machine_widget::save()
{
    QSettings settings;
    for (auto ellipse:states)
    {
        settings.setValue(ellipse.first.c_str(),ellipse.second->scenePos());
//         std::cout<<ellipse.second->scenePos().x()<<" "<<ellipse.second->scenePos().y()<<std::endl;
    }
}

state_machine_widget::~state_machine_widget()
{
    save();
}

void state_machine_widget::paintEvent(QPaintEvent* event)
{
    QGraphicsView::paintEvent(event);
}

void state_machine_widget::paintArrow(const QPointF q1, const QPointF q2)
{
    if (abs(q1.x()-q2.x())<10 || abs(q1.y()-q2.y())<10) return;
    qreal arrowSize = 10;
    QLineF main_line(q1.x(),q1.y(),q2.x(),q2.y());
    QPainterPath path;
    path.moveTo(q1.x(),q1.y());
    double dx = main_line.normalVector().unitVector().dx();
    double dy = main_line.normalVector().unitVector().dy();

    int factor = main_line.length()/3;
    path.cubicTo(q1.x()+dx*factor,q1.y()+dy*factor,q2.x()+dx*factor,q2.y()+dy*factor,q2.x(),q2.y());
    auto main_spline = Scene->addPath(path);
    main_spline->setZValue(1);
    double final_intersect_x=0, final_intersect_y=0;

    QLineF temp(q2.x()+dx*factor,q2.y()+dy*factor,q2.x()+final_intersect_x,q2.y()+final_intersect_y);
    double angle = ::acos(temp.dx() / temp.length());
    if (temp.dy() >= 0)
    {
        angle = (M_PI * 2) - angle;
    }
    QPointF arrowP1 = temp.p2() - QPointF(sin(angle + M_PI / 3) * arrowSize,
                                                       cos(angle + M_PI / 3) * arrowSize);
    
    QPointF arrowP2 = temp.p2() - QPointF(sin(angle + M_PI - M_PI / 3) * arrowSize,
                                                       cos(angle + M_PI - M_PI / 3) * arrowSize);
    Scene->addLine(arrowP1.x(),arrowP1.y(),arrowP2.x(),arrowP2.y());
    Scene->addLine(arrowP1.x(),arrowP1.y(),temp.p2().x(),temp.p2().y());
    Scene->addLine(arrowP2.x(),arrowP2.y(),temp.p2().x(),temp.p2().y());
    
}