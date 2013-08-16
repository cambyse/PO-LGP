#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"

#include "Config.h"

#include "Maze.h"
#include "KMarkovCRF.h"
#include "UTree.h"
#include "LinearQ.h"
#include "LookAheadSearch.h"
#include "DelayDistribution.h"

#include "qcustomplot.h"

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

    friend class MoveByKeys; // event filter class

    //===============================//
    // Typedefs and Member Variables //
    //===============================//

    USE_CONFIG_TYPEDEFS;

    //---------------//
    // Maze GUI etc. //
    //---------------//

    QCustomPlot * plotter;

    enum PLANNER_TYPE {
        NONE,
        OPTIMAL_PLANNER,
        SPARSE_PLANNER,
        KMDP_PLANNER,
        UTREE_PLANNER,
        UTREE_VALUE,
        LINEAR_Q_VALUE
    } planner_type;

    Ui::TestMaze_IIClass ui;
    Maze maze;

    bool record, plot;
    instance_t * current_instance;
    std::ofstream plot_file;

    QTimer * random_timer, * action_timer;

    std::vector<QString> console_history;
    size_t history_position;
    QFile history_file;

    double discount;

    //--------//
    // Models //
    //--------//

    // CRF
    KMarkovCRF crf;
    double l1_factor;

    // UTree
    UTree utree;

    // Linear_Q
    LinearQ linQ;

    //----------//
    // Planners //
    //----------//
    LookAheadSearch look_ahead_search;
    size_t max_tree_size;
    bool prune_search_tree;

    //-------//
    // Other //
    //-------//
    DelayDistribution delay_dist;
    bool target_activated;
    state_t target_state;

    //==================//
    // Member Functions //
    //==================//

    void collect_episode(const int& length);
    void update_current_instance(action_t,state_t,reward_t);
    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );
    void clear_data();

private slots:
    void render();
    void random_action();
    void choose_action();
    void process_console_input(QString sequence_input = QString(), bool sequence = false);
    void back_in_history();
    void forward_in_history();

};

// event filter for arrow keys
class MoveByKeys: public QObject {
    Q_OBJECT
public:
    MoveByKeys(TestMaze_II * m): maze(m) {}
protected:
    TestMaze_II * maze;
    bool eventFilter(QObject *obj, QEvent *event);
};

#endif // TESTMAZE_II_H
