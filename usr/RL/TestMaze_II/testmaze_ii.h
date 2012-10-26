#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include <QtGui/QWidget>
#include "ui_testmaze_ii.h"
#include "Maze.h"
#include <QTimer>


class TestMaze_II : public QWidget
{
    Q_OBJECT

public:
    TestMaze_II(QWidget *parent = 0);
    ~TestMaze_II();

private:
    Ui::TestMaze_IIClass ui;
    typedef Maze<> maze_t;
    maze_t maze;
    QTimer * random_action_timer;

private slots:
    void render();
    void random_action();
    void process_console_input();

};

#endif // TESTMAZE_II_H
