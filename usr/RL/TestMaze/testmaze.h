#ifndef TESTMAZE_H
#define TESTMAZE_H

#include <QtGui/QWidget>
#include "ui_testmaze.h"

class TestMaze : public QWidget
{
    Q_OBJECT

public:
    TestMaze(QWidget *parent = 0);
    ~TestMaze();

private:
    Ui::TestMazeClass ui;
};

#endif // TESTMAZE_H
