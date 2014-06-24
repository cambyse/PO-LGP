#ifndef MOUSEFILTER_H
#define MOUSEFILTER_H

#include <QObject>
#include <QtCore>
#include <QEvent>

class InputFilter : public QObject
{
    Q_OBJECT
public:
    explicit InputFilter(QObject *parent = 0);

signals:
    void right_click(double x, double y);
    void left_click(double x, double y);
    void scroll_up(double x, double y);
    void scroll_down(double x, double y);
    void north_key();
    void east_key();
    void south_key();
    void west_key();
    void stay_key();

protected:
    bool eventFilter(QObject *, QEvent *);

};

#endif // MOUSEFILTER_H
