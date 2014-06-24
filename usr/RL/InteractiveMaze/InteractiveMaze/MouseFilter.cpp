#include "MouseFilter.h"
#include <QDebug>
#include <QGraphicsSceneMouseEvent>

#include <iostream>
using namespace std;

MouseFilter::MouseFilter(QObject *parent) :
    QObject(parent)
{
}

bool MouseFilter::eventFilter(QObject * obj, QEvent * event)
{
    // process mouse event
    switch(event->type()) {
    case QEvent::GraphicsSceneMousePress:
    {
        QGraphicsSceneMouseEvent* mouse_event = static_cast<QGraphicsSceneMouseEvent*>(event);
        if(mouse_event==nullptr) {
            cout << "event is nullptr" << endl;
            return true;
        }
        switch(mouse_event->button()) {
        case Qt::LeftButton:
            emit left_click(mouse_event->scenePos().x(),mouse_event->scenePos().y());
            break;
        case Qt::RightButton:
            emit right_click(mouse_event->scenePos().x(),mouse_event->scenePos().y());
            break;
        default:
            break;
        }
        return true;
    }
//    case QEvent::GraphicsSceneMouseMove:
//    {
//        QGraphicsSceneMouseEvent* mouse_event = static_cast<QGraphicsSceneMouseEvent*>(event);
//        if(mouse_event==nullptr) {
//            cout << "event is nullptr" << endl;
//            return true;
//        }
//        if(mouse_event->scenePos().x()==round(mouse_event->scenePos().x()) || mouse_event->scenePos().y()==round(mouse_event->scenePos().y())) {
//#warning This is a hack to get rid of the integer coordinate events (two different floating point events remain but are similar)
//            return true;
//        }
//        switch(mouse_event->buttons()) {
//        case Qt::LeftButton:
//            emit left_mouse_move(mouse_event->scenePos().x(),mouse_event->scenePos().y());
//            cout << mouse_event->scenePos().x() << ", " << mouse_event->scenePos().y() << ", " << event << "," << obj << endl;
//            break;
//        case Qt::RightButton:
//            emit right_mouse_move(mouse_event->scenePos().x(),mouse_event->scenePos().y());
//            cout << mouse_event->scenePos().x() << ", " << mouse_event->scenePos().y() << ", " << event << "," << obj << endl;
//            break;
//        default:
//            break;
//        }
//        return true;
//    }
    case QEvent::GraphicsSceneWheel:
    {
        QGraphicsSceneWheelEvent* wheel_event = static_cast<QGraphicsSceneWheelEvent*>(event);
        if(wheel_event==nullptr) {
            cout << "event is nullptr" << endl;
            return true;
        }
        if(wheel_event->delta()>0) {
            emit scroll_up(wheel_event->scenePos().x(),wheel_event->scenePos().y());
        } else {
            emit scroll_down(wheel_event->scenePos().x(),wheel_event->scenePos().y());
        }
        return true;
    }
    default:
        break;
    }
    // standard event processing
    return QObject::eventFilter(obj, event);
}
