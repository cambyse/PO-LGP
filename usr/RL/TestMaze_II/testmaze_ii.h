#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include <QtGui/QWidget>
#include "ui_testmaze_ii.h"
#include "Maze.h"
#include "TransitionModel.h"
#include <QTimer>


class TestMaze_II : public QWidget
{
    Q_OBJECT

public:
    TestMaze_II(QWidget *parent = 0);
    ~TestMaze_II();

private:
    typedef Maze<> maze_t;
    typedef TransitionModel<maze_t::state_t,maze_t::action_t> transition_model_t;
    Ui::TestMaze_IIClass ui;
    maze_t maze;
    transition_model_t transition_model;
    QTimer * random_action_timer;

private slots:
    void render();
    void random_action();
    void process_console_input();

};

#endif // TESTMAZE_II_H
