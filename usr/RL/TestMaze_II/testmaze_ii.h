#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include <QtGui/QWidget>
#include "ui_testmaze_ii.h"
#include "Maze.h"
#include "ValueIteration.h"
#include "KMarkovCRF.h"
#include <QTimer>


class TestMaze_II : public QWidget
{
    Q_OBJECT

public:
    TestMaze_II(QWidget *parent = 0);
    ~TestMaze_II();

private:
    typedef Maze maze_t;
    typedef ValueIteration<maze_t::state_t,maze_t::action_t> value_iterationt;

    Ui::TestMaze_IIClass ui;
    maze_t maze;
    value_iterationt value_iteration_object;
    KMarkovCRF crf;
    bool record;

    QTimer * random_action_timer, * value_iteration_timer;

    void collect_episode(const int& length);
    bool arg_int(const QString& string, const int& n, int& i);
    bool arg_double(const QString& string, const int& n, double& d);
    bool arg_string(const QString& string, const int& n, QString& s);

private slots:
    void render();
    void random_action();
    void value_iteration();
    void process_console_input();

};

#endif // TESTMAZE_II_H
