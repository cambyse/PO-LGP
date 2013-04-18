#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"
#include "Maze.h"
#include "KMarkovCRF.h"
#include "LookAheadSearch.h"
#include "Data.h"
#include "Representation/Representation.h"

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

    //===============================//
    // Typedefs and Member Variables //
    //===============================//

    typedef Data::size_t        size_t;
    typedef Data::probability_t probability_t;

    //---------------//
    // Maze GUI etc. //
    //---------------//

    enum ACTION_TYPE {
        NONE,
        OPTIMAL_Q_ITERATION,
        SPARSE_Q_ITERATION,
        KMDP_Q_ITERATION,
        OPTIMAL_LOOK_AHEAD_TREE,
        SPARSE_LOOK_AHEAD_TREE,
        KMDP_LOOK_AHEAD_TREE
    } action_type;

    Ui::TestMaze_IIClass ui;
    Maze maze;

    bool record, plot;
    instance_t current_instance;
    std::ofstream plot_file;

    QTimer * random_timer, * action_timer, * value_iteration_timer;

    std::vector<QString> console_history;
    size_t history_position;

    //--------//
    // Models //
    //--------//

    KMarkovCRF crf;
    double l1_factor;

    //----------//
    // Planners //
    //----------//

    double discount;
    LookAheadSearch look_ahead_search;
    size_t max_tree_size;

    //==================//
    // Member Functions //
    //==================//

    void collect_episode(const int& length);
    void update_current_instance(action_t,state_t,reward_t);

private slots:
    void render();
    void random_action();
    void choose_action();
    void value_iteration();
    void process_console_input(QString sequence_input = QString(), bool sequence = false);
    void back_in_history();
    void forward_in_history();

};

#endif // TESTMAZE_II_H
