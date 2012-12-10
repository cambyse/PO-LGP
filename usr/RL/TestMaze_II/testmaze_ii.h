#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"
#include "Maze.h"
#include "QIteration.h"
#include "KMarkovCRF.h"

#include <QWidget>
#include <QTimer>

#include <deque>
#include <iostream>
#include <fstream>

class TestMaze_II : public QWidget
{
    Q_OBJECT

public:
    TestMaze_II(QWidget *parent = 0);
    ~TestMaze_II();

private:

    typedef Data::action_t      action_t;
    typedef Data::state_t       state_t;
    typedef Data::reward_t      reward_t;
    typedef Data::k_mdp_state_t k_mdp_state_t;
    typedef Data::data_point_t  data_point_t;

    enum ACTION_TYPE { RANDOM, OPTIMAL } action_type;
    enum ITERATION_TYPE { INT, DOUBLE } iteration_type;
    int iteration_number, iteration_counter;
    double iteration_threshold;

    Ui::TestMaze_IIClass ui;
    Maze maze;
    QIteration q_iteration_object;
    KMarkovCRF crf;
    bool record, plot;
    double l1_factor;
    std::deque<data_point_t> current_k_mdp_state_deque;
    std::ofstream plot_file;

    QTimer * action_timer, * value_iteration_timer;

    void collect_episode(const int& length);
    bool arg_int(const QString& string, const int& n, int& i);
    bool arg_double(const QString& string, const int& n, double& d);
    bool arg_string(const QString& string, const int& n, QString& s);
    void update_current_k_mdp_state(action_t,state_t,reward_t);
    k_mdp_state_t current_k_mdp_state();

private slots:
    void render();
    void choose_action();
    void value_iteration();
    void process_console_input();

};

#endif // TESTMAZE_II_H
