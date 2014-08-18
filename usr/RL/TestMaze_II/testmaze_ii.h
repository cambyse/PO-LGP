#ifndef TESTMAZE_II_H
#define TESTMAZE_II_H

#include "ui_testmaze_ii.h"

#include "Config.h"

#include "Learner/UTree.h"
#include "Learner/TemporallyExtendedLinearQ.h"
#include "Learner/TemporallyExtendedModel.h"
#include "Learner/ConjunctiveAdjacency.h"
#include "Planning/Policy.h"
#include "DelayDistribution.h"
#include "util/Commander.h"

#include "qcustomplot.h"

//#define ARMA_NO_DEBUG
#include <armadillo>

#include <QWidget>
#include <QTimer>

#include <deque>
#include <iostream>
#include <fstream>
#include <memory> // for shared_ptr
#include <map>

class Environment;

class TestMaze_II : public QWidget
{
    Q_OBJECT

public:
    TestMaze_II(QWidget *parent = 0);
    ~TestMaze_II();

    USE_CONFIG_TYPEDEFS;

private:

    friend class MoveByKeys; // event filter class

    //===============================//
    // Typedefs and Member Variables //
    //===============================//

    typedef std::map<std::tuple<observation_ptr_t,reward_ptr_t>,int> o_r_idx_map_t;

    //---------------//
    // Maze GUI etc. //
    //---------------//

    // for the plotting window
    QCustomPlot * plotter;

    // for setting the different planners
    enum PLANNER_TYPE {
        NONE,
        RANDOM,
        OPTIMAL_LOOK_AHEAD,
        UTREE_LOOK_AHEAD,
        TEM_LOOK_AHEAD,
        UTREE_VALUE,
        TEL_VALUE,
        GOAL_ITERATION
    } planner_type;

    enum TEXT_STYLE {
        INPUT_STYLE,
        OK_RESPONSE_STYLE,
        ERROR_RESPONSE_STYLE,
        NORMAL_STYLE
    };

    // the user interface
    Ui::TestMaze_IIClass ui;
    Commander::CommandCenter command_center;

    // the environment
    std::shared_ptr<Environment> environment;
    action_ptr_t action_space;             ///< Action space to be used.
    observation_ptr_t observation_space;   ///< Observation space to be used.
    reward_ptr_t reward_space;             ///< Reward space to be used.

    // current instance (essentially the same as the maze instance)
    instance_ptr_t current_instance;

    // state flags
    bool record, plot, start_new_episode, search_tree_invalid, save_png_on_transition, color_maze;

    // file for writing out transitions
    std::ofstream plot_file;

    // counter for saved png files
    unsigned int png_counter;

    // time for repeated execution of actions
    QTimer * action_timer;

    // stuff for a persistent console history
    std::vector<QString> console_history;
    large_size_t history_position;
    QFile history_file;

    // discout that is used
    double discount;

    // epsilon (randomness) that is used
    double epsilon;

    //--------//
    // Models //
    //--------//

    // L1-regularization
    double l1_factor;

    // Learners
    std::shared_ptr<UTree> utree;                     ///< UTree
    std::shared_ptr<ConjunctiveAdjacency> N_plus_TEL; ///< N+ operator for TEL
    std::shared_ptr<TemporallyExtendedLinearQ> tel;   ///< Linear-Q with TEFs
    std::shared_ptr<ConjunctiveAdjacency> N_plus_TEM; ///< N+ operator for TEM
    std::shared_ptr<TemporallyExtendedModel> tem;     ///< TEM

    //----------//
    // Planners //
    //----------//
    std::shared_ptr<Policy> policy;
    large_size_t max_tree_size;
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

    void initialize_commands();
    void to_console(QString x, TEXT_STYLE = NORMAL_STYLE, int indentation = 0);
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
    double validate_predictor_on_random_episode(int n, const Predictor& pred);
    double validate_predictor_on_training_episode(const Predictor& pred);
    void get_TEM_transition_matrix_and_o_r_index_map(arma::mat& T, o_r_idx_map_t& o_r_idx_map) const;
    bool get_stationary_distribution(const arma::mat& T, arma::vec& stat_dist) const;

private slots:
    void render_update();
    void choose_action();
    void process_console_input();
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
