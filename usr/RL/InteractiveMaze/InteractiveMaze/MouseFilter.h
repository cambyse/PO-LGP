#ifndef MOUSEFILTER_H
#define MOUSEFILTER_H

#include <QObject>
#include <QtCore>
#include <QEvent>

class MouseFilter : public QObject
{
    Q_OBJECT
public:
    explicit MouseFilter(QObject *parent = 0);

signals:
    void right_click(double x, double y);
    void left_click(double x, double y);
    void scroll_up(double x, double y);
    void scroll_down(double x, double y);
//    void right_mouse_move(double x, double y);
//    void left_mouse_move(double x, double y);

protected:
    bool eventFilter(QObject *, QEvent *);

};

#endif // MOUSEFILTER_H
