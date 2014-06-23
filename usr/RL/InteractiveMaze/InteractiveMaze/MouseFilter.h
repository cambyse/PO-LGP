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
    void right_click(int x, int y);
    void left_click(int x, int y);
    void scroll_up(int x, int y);
    void scroll_down(int x, int y);
    void right_mouse_move(int x, int y);
    void left_mouse_move(int x, int y);

protected:
    bool eventFilter(QObject *, QEvent *);

};

#endif // MOUSEFILTER_H
