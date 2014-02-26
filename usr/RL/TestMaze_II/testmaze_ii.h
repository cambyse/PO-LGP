#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"

#include "Config.h"

#include "KMarkovCRF.h"
#include "UTree.h"
#include "LinearQ.h"
#include "Planning/Policy.h"
#include "DelayDistribution.h"

#include "qcustomplot.h"

#include <QWidget>
#include <QTimer>

#include <deque>
#include <iostream>
#include <fstream>
#include <memory> // for shared_ptr

class Environment;

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

    // for setting the different planners
    enum PLANNER_TYPE {
        NONE,
        OPTIMAL_LOOK_AHEAD,
        SPARSE_LOOK_AHEAD,
        KMDP_LOOK_AHEAD,
        UTREE_LOOK_AHEAD,
        UTREE_VALUE,
        LINEAR_Q_VALUE,
        GOAL_ITERATION
    } planner_type;

    // the user interface
    Ui::TestMaze_IIClass ui;

    // the environment
    std::shared_ptr<Environment> environment;
    action_ptr_t action_space;             ///< Action space to be used.
    observation_ptr_t observation_space;   ///< Observation space to be used.
    reward_ptr_t reward_space;             ///< Reward space to be used.

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

    // discout that is used
    double discount;

    // epsion (randomness) that is used
    double epsilon;

    //--------//
    // Models //
    //--------//

    // L1-regularization
    double l1_factor;

    // CRF
    std::shared_ptr<KMarkovCRF> crf;

    // UTree
    std::shared_ptr<UTree> utree;

    // Linear_Q
    std::shared_ptr<LinearQ> linQ;

    //----------//
    // Planners //
    //----------//
    std::shared_ptr<Policy> policy;
    size_t max_tree_size;
    bool prune_search_tree;

    //-------//
    // Other //
    //-------//
    DelayDistribution delay_dist;
    bool goal_activated;
    observation_ptr_t goal_state;

    //==================//
    // Member Functions //
    //==================//

    void collect_episode(const int& length);
    void update_current_instance(action_ptr_t, observation_ptr_t, reward_ptr_t, bool invalidate_search_tree = true);
    void add_action_observation_reward_tripel(
            const action_ptr_t& action,
            const observation_ptr_t& observation,
            const reward_ptr_t& reward
    );
    void clear_data();
    void fully_expand_utree();
    void save_to_png(QString file_name) const;
    void perform_transition(const action_ptr_t& action);
    void perform_transition(const action_ptr_t& action, observation_ptr_t& observation_to, reward_ptr_t& reward);
    void change_environment(std::shared_ptr<Environment> new_environment);
    void clear_all_learners();
    void set_policy();

private slots:
    void render_update();
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
