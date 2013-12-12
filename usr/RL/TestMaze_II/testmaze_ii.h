#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"

#include "Config.h"

#include "Maze/Maze.h"
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

    // for the plotting window
    QCustomPlot * plotter;

    // for setting the different planers
    enum PLANNER_TYPE {
        NONE,
        OPTIMAL_PLANNER,
        SPARSE_PLANNER,
        KMDP_PLANNER,
        UTREE_PLANNER,
        UTREE_VALUE,
        LINEAR_Q_VALUE
    } planner_type;

    // the user interface
    Ui::TestMaze_IIClass ui;

    // the maze / the world
    Maze maze;

    // current instance (essentially the same as the maze instance)
    instance_t * current_instance;

    // state flags
    bool record, plot, start_new_episode, search_tree_invalid, save_png_on_transition;

    // file for writing out transitions
    std::ofstream plot_file;

    // counter for saved png files
    unsigned int png_counter;

    // time for repeated execution of actions
    QTimer * random_timer, * action_timer;

    // stuff for a persistent console history
    std::vector<QString> console_history;
    size_t history_position;
    QFile history_file;

    // this discout that is used
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
    observation_t target_state;

    //==================//
    // Member Functions //
    //==================//

    void collect_episode(const int& length);
    void update_current_instance(action_t, observation_t, reward_t, bool invalidate_search_tree = true);
    void add_action_observation_reward_tripel(
            const action_t& action,
            const observation_t& observation,
            const reward_t& reward
    );
    void clear_data();
    void fully_expand_utree();
    void save_to_png(QString file_name) const;
    void perform_transition(const action_t& action);
    void perform_transition(const action_t& action, observation_t& observation_to, reward_t& reward);

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
