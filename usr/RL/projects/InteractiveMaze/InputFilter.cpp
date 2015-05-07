#include "InputFilter.h"
#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>

#include <iostream>
using namespace std;

InputFilter::InputFilter(QObject *parent) :
    QObject(parent)
{
}

bool InputFilter::eventFilter(QObject * obj, QEvent * event)
{
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
    case QEvent::KeyPress:
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        switch(keyEvent->key()) {
        case Qt::Key_Up:
            emit north_key();
            return true;
        case Qt::Key_Right:
            emit east_key();
            return true;
        case Qt::Key_Down:
            emit south_key();
            return true;
        case Qt::Key_Left:
            emit west_key();
            return true;
        case Qt::Key_Space:
            emit stay_key();
            return true;
        default:
            break;
        }
    }
    default:
        break;
    }
    // standard event processing
    return QObject::eventFilter(obj, event);
}
