#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"
#include "Maze.h"
#include "QIteration.h"
#include "KMarkovCRF.h"
#include "KMDPState.h"
#include "LookAheadSearch.h"
#include "MCTS.h"

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
    typedef Data::action_t      action_t;
    typedef Data::state_t       state_t;
    typedef Data::reward_t      reward_t;
    typedef Data::probability_t probability_t;
    typedef Data::k_mdp_state_t k_mdp_state_t;
    typedef Data::data_point_t  data_point_t;

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
    KMDPState current_k_mdp_state;
    std::ofstream plot_file;

    QTimer * random_timer, * action_timer, * value_iteration_timer;

    //--------//
    // Models //
    //--------//

    KMarkovCRF crf;
    double l1_factor;

    //----------//
    // Planners //
    //----------//

    double discount;

    QIteration * q_iteration_object;
    bool q_iteration_available;
    int iteration_number;
    double iteration_threshold;
    enum ITERATION_TYPE { INT, DOUBLE } iteration_type;

    LookAheadSearch look_ahead_search;
//    probability_t probability_threshold;
//    size_t tree_depth;

    //==================//
    // Member Functions //
    //==================//

    void collect_episode(const int& length);
    void update_current_k_mdp_state(action_t,state_t,reward_t);

private slots:
    void render();
    void random_action();
    void choose_action();
    void value_iteration();
    void process_console_input();

};

#endif // TESTMAZE_II_H
