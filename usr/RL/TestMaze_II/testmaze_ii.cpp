#include "testmaze_ii.h"
#include "Data.h"
#include "util.h"

#define DEBUG_LEVEL 1
#include "debug.h"

using util::arg_int;
using util::arg_double;
using util::arg_string;

TestMaze_II::TestMaze_II(QWidget *parent)
    : QWidget(parent),
      action_type(NONE),
      maze(0.0),
      record(false), plot(false),
      current_k_mdp_state(),
      random_timer(nullptr), action_timer(nullptr), value_iteration_timer(nullptr),
      l1_factor(0),
      discount(0.9),
      q_iteration_object(nullptr), q_iteration_available(false),
      iteration_number(0), iteration_threshold(1), iteration_type(INT),
      look_ahead_tree(discount),
      probability_threshold(0),
      tree_depth(2*(Data::maze_x_dim-1)+2*(Data::maze_y_dim-1))
{
    // initialize UI
    ui.setupUi(this);

    // focus on command line
    ui._wConsoleInput->setFocus();

    // set console welcome message
    ui._wConsoleOutput->setPlainText("    Please enter your commands (type 'help' for an overview)");

    // initialize timers
    random_timer = new QTimer(this);
    connect(random_timer, SIGNAL(timeout()), this, SLOT(random_action()));
    action_timer = new QTimer(this);
    connect(action_timer, SIGNAL(timeout()), this, SLOT(choose_action()));
    value_iteration_timer = new QTimer(this);
    connect(value_iteration_timer, SIGNAL(timeout()), this, SLOT(value_iteration()));

    // initialize model for model selection
    if(Data::maze_x_dim<3 && Data::maze_y_dim<3) {
        q_iteration_object = new QIteration();
        maze.initialize_predictions(*q_iteration_object);
        q_iteration_object->set_discount(discount);
        q_iteration_available = true;
    } else {
        q_iteration_available = false;
    }

    // initialize display
    maze.render_initialize(ui.graphicsView);

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render()));

}

TestMaze_II::~TestMaze_II() {
    delete random_timer;
    delete action_timer;
    delete value_iteration_timer;
    delete q_iteration_object;
    plot_file.close();
}

void TestMaze_II::collect_episode(const int& length) {
    for(int idx=0; idx<length; ++idx) {
        action_t action = (action_t)(rand()%Data::action_n);
        state_t state_to;
        reward_t reward;
        maze.perform_transition(action,state_to,reward);
        update_current_k_mdp_state(action,state_to,reward);
        crf.add_action_state_reward_tripel(action,state_to,reward);
    }
}

void TestMaze_II::update_current_k_mdp_state(action_t action, state_t state, reward_t reward) {
    current_k_mdp_state.new_state(action,state,reward);
    if(plot) {
        plot_file << action << " " << state << " " << reward << std::endl;
    }
}

void TestMaze_II::render() {
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::random_action() {
    action_t action = (action_t)(rand()%Data::action_n);
    state_t state_to;
    reward_t reward;
    maze.perform_transition(action,state_to,reward);
    update_current_k_mdp_state(action,state_to,reward);
    if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::choose_action() {
    action_t action;
    switch(action_type) {
    case OPTIMAL_Q_ITERATION:
    case SPARSE_Q_ITERATION:
    case KMDP_Q_ITERATION:
        if(q_iteration_available) {
            action = q_iteration_object->optimal_action(current_k_mdp_state.get_k_mdp_state());
        } else {
            DEBUG_OUT(0, "Q-Iteration only available for 2x2 mazes --> choosing STAY");
            action = Data::STAY;
        }
        break;
    case OPTIMAL_LOOK_AHEAD_TREE:
        look_ahead_tree.build_tree<Maze>(
                current_k_mdp_state,
                tree_depth,
                probability_threshold,
                maze,
                maze.get_prediction_ptr()
        );
        action = look_ahead_tree.get_best_action();
        look_ahead_tree.clear_tree();
        break;
    case SPARSE_LOOK_AHEAD_TREE:
        look_ahead_tree.build_tree<KMarkovCRF>(
                current_k_mdp_state,
                tree_depth,
                probability_threshold,
                crf,
                crf.get_prediction_ptr()
        );
        action = look_ahead_tree.get_best_action();
        look_ahead_tree.clear_tree();
        break;
    case KMDP_LOOK_AHEAD_TREE:
        action = Data::STAY;
        DEBUG_OUT(0, "Not implemented as yet --> choosing STAY");
        break;
    default:
        action = Data::STAY;
        DEBUG_OUT(0,"Error: undefined action type --> choosing stay");
        break;
    }
    state_t state_to;
    reward_t reward;
    maze.perform_transition(action,state_to,reward);
    update_current_k_mdp_state(action,state_to,reward);
    if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::value_iteration() {
    if(q_iteration_available) {
        reward_t max_value_diff = q_iteration_object->iterate();
        switch(iteration_type) {
        case INT:
            --iteration_number;
            if(iteration_number<=0) {
                value_iteration_timer->stop();
            }
            break;
        case DOUBLE:
            if(max_value_diff<iteration_threshold) {
                value_iteration_timer->stop();
            }
            break;
        default:
            DEBUG_OUT(0,"Error: invalid iteration type");
            break;
        }
    } else {
        DEBUG_OUT(0, "Q-Iteration only available for 2x2 mazes.");
    }
}

void TestMaze_II::process_console_input() {
    QString input = ui._wConsoleInput->text();
    ui._wConsoleInput->setText("");
    ui._wConsoleOutput->appendPlainText(input);

    // help strings
    QString headline_s(                 "Available commands:\n    COMMAND . . .  ARGUMENTS . . . . . . . . . . . . . . . . -> ACTION");

    QString general_s(                "\n    -------------------------General---------------------------");
    QString help_s(                     "    help  / h. . . . . . . . . . . . . . . . . . . . . . . . -> this help");
    QString exit_s(                     "    exit/quit/q. . . . . . . . . . . . . . . . . . . . . . . -> quit application");
    QString set_s(                      "    set/unset. . . . . . . . . <string>. . . . . . . . . . . -> set/unset options:");
    QString option_1_s(                 "                               record. . . . . . . . . . . . -> start/stop recording movements");
    QString option_2_s(                 "                               plot. . . . . . . . . . . . . -> write transitions into data file for plotting");

    QString maze_s(                   "\n    ---------------------------Maze---------------------------");
    QString left_s(                     "    left  / l. . . . . . . . . . . . . . . . . . . . . . . . -> move left");
    QString right_s(                    "    right / r. . . . . . . . . . . . . . . . . . . . . . . . -> move right");
    QString up_s(                       "    up    / u. . . . . . . . . . . . . . . . . . . . . . . . -> move up");
    QString down_s(                     "    down  / d. . . . . . . . . . . . . . . . . . . . . . . . -> move down");
    QString stay_s(                     "    stay  / s. . . . . . . . . . . . . . . . . . . . . . . . -> stay-action");
    QString move_s(                     "    move . . . . . . . . . . . [<int>|stop]. . . . . . . . . -> start/stop moving using planner");
    QString random_s(                   "    random . . . . . . . . . . [<int>|stop]. . . . . . . . . -> start/stop moving randomly");
    QString delay_s(                    "    delay. . . . . . . . . . . [<int>] . . . . . . . . . . . -> get [set] reward delay");
    QString epsilon_s(                  "    epsilon. . . . . . . . . . [<double>]. . . . . . . . . . -> get [set] random transition probability");

    QString learning_s(               "\n    ----------------------Model Learning----------------------");
    QString episode_s(                  "    episode / e. . . . . . . . [<int>|clear,c] . . . . . . . -> record length <int> episode or clear data");
    QString optimize_s(                 "    optimize / o . . . . . . . [check, c]. . . . . . . . . . -> optimize CRF [check derivatives]");
    QString score_s(                    "    score. . . . . . . . . . . <int> . . . . . . . . . . . . -> score compound features with distance <int> by gradient");
    QString add_s(                      "    add. . . . . . . . . . . . <int> . . . . . . . . . . . . -> add <int> highest scored compound features to active (0 for all non-zero scored)");
    QString erase_s(                    "    erase. . . . . . . . . . . . . . . . . . . . . . . . . . -> erase features with zero weight");
    QString l1_s(                       "    l1 . . . . . . . . . . . . <double>. . . . . . . . . . . -> coefficient for L1 regularization");
    QString evaluate_s(                 "    evaluate . . . . . . . . . . . . . . . . . . . . . . . . -> evaluate features at current point");
    QString validate_s(                 "    validate / v . . . . . . . {crf,kmdp}[exact|mc <int>]. . -> validate CRF or k-MDP model using exact (default) or Monte Carlo (with <int> samples) computation of the KL-divergence");

    QString planning_s(               "\n    -------------------------Planning--------------------------");
    QString iterate_s(                  "    iterate / i. . . . . . . . [<int>|<double>,stop] . . . . -> run value iteration <int> times / until max diff small than <double> / stop running");
    QString discount_s(                 "    discount . . . . . . . . . [<double>]. . . . . . . . . . -> get [set] discount");
    QString optimal_iteration_s(        "    optimal-iteration. . . . . . . . . . . . . . . . . . . . -> use known predictions for value iteration");
    QString sparse_iteration_s(         "    sparse-iteration . . . . . . . . . . . . . . . . . . . . -> use sparse model for value iteration");
    QString kmdp_iteration_s(           "    kmdp-iteration . . . . . . . . . . . . . . . . . . . . . -> use k-MDP model for value iteration");
    QString optimal_look_ahead_tree_s(  "    optimal-look-ahead-tree. . [<int>[<double>]] . . . . . . -> use known predictions for Look-Ahead-Tree [ depth [ threshold ] ]");
    QString sparse_look_ahead_tree_s(   "    sparse-look-ahead-tree . . [<int>[<double>]] . . . . . . -> use sparse model for Look-Ahead-Tree [ depth [ threshold ] ]");
    QString kmdp_look_ahead_tree_s(     "    kmdp-look-ahead-tree . . . [<int>[<double>]] . . . . . . -> use k-MDP model for Look-Ahead-Tree [ depth [ threshold ] ]");

    set_s += "\n" + option_1_s;
    set_s += "\n" + option_2_s;

    QString invalid_s( "    invalid arguments" );

    // getting input arguments
    std::vector<QString> str_args;
    QString tmp_s;
    std::vector<int> int_args;
    std::vector<bool> int_args_ok;
    int tmp_i;
    std::vector<double> double_args;
    std::vector<bool> double_args_ok;
    double tmp_d;
    for(int arg_idx=0; arg_string(input,arg_idx,tmp_s) && tmp_s!=""; ++arg_idx) {

        str_args.push_back(tmp_s);

        int_args_ok.push_back( arg_int(input,arg_idx,tmp_i) );
        int_args.push_back( tmp_i );

        double_args_ok.push_back( arg_double(input,arg_idx,tmp_d) );
        double_args.push_back( tmp_d );
    }

    // process input
    if(str_args.size()>0) {
        if(str_args[0]=="help" || str_args[0]=="h") { // help
            // Headline
            ui._wConsoleOutput->appendPlainText( headline_s );
            // General
            ui._wConsoleOutput->appendPlainText( general_s );
            ui._wConsoleOutput->appendPlainText( help_s );
            ui._wConsoleOutput->appendPlainText( exit_s );
            ui._wConsoleOutput->appendPlainText( set_s );
            // Maze
            ui._wConsoleOutput->appendPlainText( maze_s );
            ui._wConsoleOutput->appendPlainText( left_s );
            ui._wConsoleOutput->appendPlainText( right_s );
            ui._wConsoleOutput->appendPlainText( up_s );
            ui._wConsoleOutput->appendPlainText( down_s );
            ui._wConsoleOutput->appendPlainText( stay_s );
            ui._wConsoleOutput->appendPlainText( move_s );
            ui._wConsoleOutput->appendPlainText( random_s );
            ui._wConsoleOutput->appendPlainText( delay_s );
            ui._wConsoleOutput->appendPlainText( epsilon_s );
            // Learning
            ui._wConsoleOutput->appendPlainText( learning_s );
            ui._wConsoleOutput->appendPlainText( episode_s );
            ui._wConsoleOutput->appendPlainText( optimize_s );
            ui._wConsoleOutput->appendPlainText( score_s );
            ui._wConsoleOutput->appendPlainText( add_s );
            ui._wConsoleOutput->appendPlainText( erase_s );
            ui._wConsoleOutput->appendPlainText( l1_s );
            ui._wConsoleOutput->appendPlainText( evaluate_s );
            ui._wConsoleOutput->appendPlainText( validate_s );
            // Planning
            ui._wConsoleOutput->appendPlainText( planning_s );
            ui._wConsoleOutput->appendPlainText( iterate_s );
            ui._wConsoleOutput->appendPlainText( discount_s );
            ui._wConsoleOutput->appendPlainText( optimal_iteration_s );
            ui._wConsoleOutput->appendPlainText( sparse_iteration_s );
            ui._wConsoleOutput->appendPlainText( kmdp_iteration_s );
            ui._wConsoleOutput->appendPlainText( optimal_look_ahead_tree_s );
            ui._wConsoleOutput->appendPlainText( sparse_look_ahead_tree_s );
            ui._wConsoleOutput->appendPlainText( kmdp_look_ahead_tree_s );
        } else if(str_args[0]=="left" || str_args[0]=="l") { // left
            action_t action = Data::LEFT;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_k_mdp_state(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="right" || str_args[0]=="r") { // right
            action_t action = Data::RIGHT;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_k_mdp_state(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="up" || str_args[0]=="u") { // up
            action_t action = Data::UP;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_k_mdp_state(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="down" || str_args[0]=="d") { // down
            action_t action = Data::DOWN;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_k_mdp_state(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="stay" || str_args[0]=="s") { // stay
            action_t action = Data::STAY;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_k_mdp_state(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="move") { // start/stop moving
            if(str_args.size()==1) {
                choose_action();
            } else if(str_args[1]=="stop") {
                action_timer->stop();
            } else if(int_args_ok[1] && int_args[1]>=0){
                action_timer->stop();
                action_timer->start(int_args[1]);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( move_s );
            }
        } else if(str_args[0]=="random") { // start/stop moving
            if(str_args.size()==1) {
                random_action();
            } else if(str_args[1]=="stop") {
                random_timer->stop();
            } else if(int_args_ok[1] && int_args[1]>=0){
                random_timer->stop();
                random_timer->start(int_args[1]);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( random_s );
            }
        } else if(str_args[0]=="delay") { // set time delay for rewards
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText("    " + QString::number(maze.get_time_delay()));
            } else if(int_args_ok[1] && int_args[1]>=0){
                maze.set_time_delay(int_args[1]);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( delay_s );
            }
        } else if(str_args[0]=="iterate" || str_args[0]=="i") { // value iteration
            if(str_args.size()==1) {
                value_iteration();
            } else if(str_args[1]=="stop") {
                value_iteration_timer->stop();
            } else if(int_args_ok[1] && int_args[1]>=0){
                value_iteration_timer->stop();
                iteration_type = INT;
                iteration_number = int_args[1];
                value_iteration_timer->start();
            } else if(double_args_ok[1] && double_args[1]>=0){
                value_iteration_timer->stop();
                iteration_type = DOUBLE;
                iteration_threshold = double_args[1];
                value_iteration_timer->start();
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( iterate_s );
            }
        } else if(str_args[0]=="episode" || str_args[0]=="e") { // record episode
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( episode_s );
            } else if( str_args[1]=="clear" || str_args[1]=="c" ) {
                crf.clear_data();
            } else if(int_args_ok[1] && int_args[1]>=0){
                collect_episode(int_args[1]);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( episode_s );
            }
        } else if(str_args[0]=="optimize" || str_args[0]=="o") { // optimize CRF
            if(str_args.size()==1) {
                crf.optimize_model(l1_factor);
            } else if(str_args[1]=="check" || str_args[1]=="c") {
                crf.check_derivatives(3,10,1e-6,1e-3);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( optimize_s );
            }
        } else if(str_args[0]=="epsilon") {
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText("    " + QString::number(maze.get_epsilon()));
            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                maze.set_epsilon(double_args[1]);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( epsilon_s );
            }
        } else if(str_args[0]=="discount") {
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText("    " + QString::number(discount));
            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                discount = double_args[1];
                look_ahead_tree.set_discount(discount);
                if(q_iteration_available) {
                    q_iteration_object->set_discount(discount);
                }
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( discount_s );
            }
        } else if(str_args[0]=="evaluate") {
            crf.evaluate_features();
        } else if(str_args[0]=="validate" || str_args[0]=="v") {
            if(str_args.size()==1 ) {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( validate_s );
            } else if(str_args[1]=="crf") {
                if(str_args.size()==2 || str_args[2]=="exact") {
                    probability_t kl = maze.validate_model<KMarkovCRF>(
                            crf,
                            crf.get_prediction_ptr(),
                            Maze::EXACT_VALIDATION
                    );
                    ui._wConsoleOutput->appendPlainText(QString("    Exact KL-Divergence = %1").arg(kl));
                } else if(str_args[2]=="mc") {
                    if( str_args.size()>3 && int_args_ok[3] && int_args[3]>0 ) {
                        probability_t kl = maze.validate_model<KMarkovCRF>(
                                crf,
                                crf.get_prediction_ptr(),
                                Maze::MONTE_CARLO_VALIDATION,
                                int_args[3]
                        );
                        ui._wConsoleOutput->appendPlainText(QString("    MC KL-Divergence = %1 (%2 samples)").arg(kl).arg(int_args[3]));
                    } else {
                        ui._wConsoleOutput->appendPlainText( "    Please specify a valid sample size" );
                    }
                } else {
                    ui._wConsoleOutput->appendPlainText( invalid_s );
                    ui._wConsoleOutput->appendPlainText( validate_s );
                }
            } else if(str_args[1]=="kmdp") {
                if(str_args.size()==2 || str_args[2]=="exact") {
                    probability_t kl = maze.validate_model<KMarkovCRF>(
                            crf,
                            crf.get_prediction_ptr(),
                            Maze::EXACT_VALIDATION
                    );
                    ui._wConsoleOutput->appendPlainText(QString("    Exact KL-Divergence = %1").arg(kl));
                } else if(str_args[2]=="mc") {
                    if( str_args.size()>3 && int_args_ok[3] && int_args[3]>0 ) {
                        probability_t kl = maze.validate_model<KMarkovCRF>(
                                crf,
                                crf.get_prediction_ptr(),
                                Maze::MONTE_CARLO_VALIDATION,
                                int_args[3]
                        );
                        ui._wConsoleOutput->appendPlainText(QString("    MC KL-Divergence = %1 (%2 samples)").arg(kl).arg(int_args[3]));
                    } else {
                        ui._wConsoleOutput->appendPlainText( "    Please specify a valid sample size" );
                    }
                } else {
                    ui._wConsoleOutput->appendPlainText( invalid_s );
                    ui._wConsoleOutput->appendPlainText( validate_s );
                }
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( validate_s );
            }
        } else if(str_args[0]=="l1") {
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText(QString("    %1").arg(l1_factor));
            } else if(double_args_ok[1] && double_args[1]>=0) {
                l1_factor = double_args[1];
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( l1_s );
            }
        } else if(str_args[0]=="score") {
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText(score_s);
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                crf.score_features_by_gradient(int_args[1]);
                crf.sort_scored_features();
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( score_s );
            }
        } else if(str_args[0]=="add") {
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( add_s );
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                crf.add_compound_features_to_active(int_args[1]);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( add_s );
            }
        } else if(str_args[0]=="erase") {
            crf.erase_zero_features();
        } else if(str_args[0]=="exit" || str_args[0]=="quit" || str_args[0]=="q") { // quit application
            QApplication::quit();
        } else if(str_args[0]=="optimal-look-ahead-tree") {
            action_type = OPTIMAL_LOOK_AHEAD_TREE;
            if(str_args.size()>1 && int_args_ok[1] && int_args[1]>0) {
                tree_depth = int_args[1];
                if(str_args.size()>2 && double_args_ok[2] && double_args[2]>=0) {
                    probability_threshold = double_args[2];
                } else {
                    ui._wConsoleOutput->appendPlainText( "    Please specify a valid probability threshold" );
                }
            } else {
                ui._wConsoleOutput->appendPlainText( "    Please specify a valid tree depth" );
            }
        } else if(str_args[0]=="sparse-look-ahead-tree") {
            action_type = SPARSE_LOOK_AHEAD_TREE;
            if(str_args.size()>1 && int_args_ok[1] && int_args[1]>0) {
                tree_depth = int_args[1];
                if(str_args.size()>2 && double_args_ok[2] && double_args[2]>=0) {
                    probability_threshold = double_args[2];
                } else {
                    ui._wConsoleOutput->appendPlainText( "    Please specify a valid probability threshold" );
                }
            } else {
                ui._wConsoleOutput->appendPlainText( "    Please specify a valid tree depth" );
            }
        } else if(str_args[0]=="kmdp-look-ahead-tree") {
            action_type = KMDP_LOOK_AHEAD_TREE;
            if(str_args.size()>1 && int_args_ok[1] && int_args[1]>0) {
                tree_depth = int_args[1];
                if(str_args.size()>2 && double_args_ok[2] && double_args[2]>=0) {
                    probability_threshold = double_args[2];
                } else {
                    ui._wConsoleOutput->appendPlainText( "    Please specify a valid probability threshold" );
                }
            } else {
                ui._wConsoleOutput->appendPlainText( "    Please specify a valid tree depth" );
            }
        } else if(str_args[0]=="optimal-iteration") {
            if(q_iteration_available) {
                q_iteration_object->clear();
                maze.initialize_predictions(*q_iteration_object);
                ui._wConsoleOutput->appendPlainText("    initialized prediction matrix with true values");
                action_type = OPTIMAL_Q_ITERATION;
            } else {
                ui._wConsoleOutput->appendPlainText( "    Q-Iteration only available for 2x2 mazes.");
            }
        } else if(str_args[0]=="sparse-iteration") {
            if(q_iteration_available) {
                q_iteration_object->clear();
                crf.initialize_sparse_predictions(*q_iteration_object);
                ui._wConsoleOutput->appendPlainText("    initialized prediction matrix values from sparse model");
                action_type = SPARSE_Q_ITERATION;
            } else {
                ui._wConsoleOutput->appendPlainText( "    Q-Iteration only available for 2x2 mazes.");
            }
        } else if(str_args[0]=="kmdp-iteration") {
            if(q_iteration_available) {
                q_iteration_object->clear();
                crf.initialize_kmdp_predictions(*q_iteration_object);
                ui._wConsoleOutput->appendPlainText("    initialized prediction matrix values with relative frequencies");
                action_type = KMDP_Q_ITERATION;
            } else {
                ui._wConsoleOutput->appendPlainText( "    Q-Iteration only available for 2x2 mazes.");
            }
        } else if(str_args[0]=="set" || str_args[0]=="unset") { // set option
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( set_s );
            } else if(str_args[1]=="record") {
                    record = str_args[0]=="set";
                    if(record) {
                        ui._wConsoleOutput->appendPlainText( "    record on" );
                    } else {
                        ui._wConsoleOutput->appendPlainText( "    record off" );
                    }
            } else if(str_args[1]=="plot") {
                plot = str_args[0]=="set";
                if(plot) {
                    // open plot file
                    plot_file.open("plot_file.txt");
                    plot_file << "# action state reward" << std::endl;
                    ui._wConsoleOutput->appendPlainText( "    plot on" );
                } else {
                    // close plot file
                    plot_file.close();
                    ui._wConsoleOutput->appendPlainText( "    plot off" );
                }
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( set_s );
            }
        } else {
            ui._wConsoleOutput->appendPlainText("    unknown command");
        }
    }
}
