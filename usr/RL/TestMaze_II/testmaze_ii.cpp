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
//      action_type(NONE),
      action_type(OPTIMAL_LOOK_AHEAD_TREE),
      maze(0.0),
      record(false), plot(false),
      current_instance(nullptr),
      random_timer(nullptr), action_timer(nullptr),
      console_history(1,"END OF HISTORY"),
      history_position(0),
      l1_factor(0),
      discount(0.7),
      look_ahead_search(discount),
//      probability_threshold(0),
//      tree_depth(2*(Data::maze_x_size-1)+2*(Data::maze_y_size-1))
      max_tree_size(10000)
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

    // initialize display
    maze.render_initialize(ui.graphicsView);

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render()));

}

TestMaze_II::~TestMaze_II() {
    delete random_timer;
    delete action_timer;
    delete current_instance;
    plot_file.close();
}

void TestMaze_II::collect_episode(const int& length) {
    for(int idx=0; idx<length; ++idx) {
        action_t action = (action_t)(action_t::random_action());
        state_t state_to;
        reward_t reward;
        maze.perform_transition(action,state_to,reward);
        update_current_instance(action,state_to,reward);
        crf.add_action_state_reward_tripel(action,state_to,reward);
    }
}

void TestMaze_II::update_current_instance(action_t action, state_t state, reward_t reward) {
    if(current_instance==nullptr) {
        current_instance = instance_t::create(action,state,reward);
    } else {
        current_instance = current_instance->append_instance(action,state,reward);
    }
    if(plot) {
        plot_file << action << " " << state << " " << reward << std::endl;
    }
}

void TestMaze_II::render() {
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::random_action() {
    action_t action = (action_t)(action_t::random_action());
    state_t state_to;
    reward_t reward;
    maze.perform_transition(action,state_to,reward);
    update_current_instance(action,state_to,reward);
    if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::choose_action() {
    action_t action;
    switch(action_type) {
    case OPTIMAL_LOOK_AHEAD_TREE:
        look_ahead_search.clear_tree();
        look_ahead_search.build_tree<Maze>(
                current_instance,
                maze,
                maze.get_prediction_ptr(),
                max_tree_size
        );
        action = look_ahead_search.get_optimal_action();
        break;
    case SPARSE_LOOK_AHEAD_TREE:
        look_ahead_search.clear_tree();
        look_ahead_search.build_tree<KMarkovCRF>(
                current_instance,
                crf,
                crf.get_prediction_ptr(),
                max_tree_size
        );
        action = look_ahead_search.get_optimal_action();
        break;
    case KMDP_LOOK_AHEAD_TREE:
        crf.update_prediction_map();
        look_ahead_search.clear_tree();
        look_ahead_search.build_tree<KMarkovCRF>(
                current_instance,
                crf,
                crf.get_kmdp_prediction_ptr(),
                max_tree_size
        );
        action = look_ahead_search.get_optimal_action();
        break;
    default:
        action = action_t::STAY;
        DEBUG_OUT(0,"Error: undefined action type --> choosing STAY");
        break;
    }
    state_t state_to;
    reward_t reward;
    maze.perform_transition(action,state_to,reward);
    update_current_instance(action,state_to,reward);
    if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::process_console_input(QString sequence_input, bool sequence) {

    QString input;

    // deal with sequences
    if(sequence) {
        input = sequence_input;
    } else {
        input = ui._wConsoleInput->text();
        ui._wConsoleInput->setText("");
        ui._wConsoleOutput->appendPlainText(input);
        console_history.push_back(input);
        history_position = console_history.size();

        // check for sequence
        QStringList command_list = input.split(";");
        if(command_list.length()==0) {
            DEBUG_OUT(0, "Error: Empty command list");
            return;
        } else if(command_list.length()>1) {
            for(int command_idx=0; command_idx<command_list.length(); ++command_idx) {
                process_console_input(command_list.at(command_idx),true);
            }
            return;
        } else {
            input = command_list.at(0);
        }
    }

    // remove leading and trailing space
    while(input.startsWith(' ')) {
        input.remove(0,1);
    }
    while(input.endsWith(' ')) {
        input.chop(1);
    }

    // help strings
    QString headline_s(                         "Available commands:\n    COMMAND . . .  ARGUMENTS . . . . . . . . . . . . . . . . -> ACTION");

    QString general_s(                        "\n    -------------------------General---------------------------");
    QString help_s(                             "    help  / h. . . . . . . . . . . . . . . . . . . . . . . . -> this help");
    QString exit_s(                             "    exit/quit/q. . . . . . . . . . . . . . . . . . . . . . . -> quit application");
    QString set_s(                              "    set/unset. . . . . . . . . <string>. . . . . . . . . . . -> set/unset options:");
    QString option_1_s(                         "                               record. . . . . . . . . . . . -> start/stop recording movements");
    QString option_2_s(                         "                               plot. . . . . . . . . . . . . -> write transitions into data file for plotting");

    QString maze_s(                           "\n    ---------------------------Maze---------------------------");
    QString left_s(                             "    left  / l. . . . . . . . . . . . . . . . . . . . . . . . -> move left");
    QString right_s(                            "    right / r. . . . . . . . . . . . . . . . . . . . . . . . -> move right");
    QString up_s(                               "    up    / u. . . . . . . . . . . . . . . . . . . . . . . . -> move up");
    QString down_s(                             "    down  / d. . . . . . . . . . . . . . . . . . . . . . . . -> move down");
    QString stay_s(                             "    stay  / s. . . . . . . . . . . . . . . . . . . . . . . . -> stay-action");
    QString move_s(                             "    move . . . . . . . . . . . [<int>|stop]. . . . . . . . . -> start/stop moving using planner");
    QString random_s(                           "    random . . . . . . . . . . [<int>|stop]. . . . . . . . . -> start/stop moving randomly");
    QString delay_s(                            "    delay. . . . . . . . . . . [<int>] . . . . . . . . . . . -> get [set] reward delay");
    QString epsilon_s(                          "    epsilon. . . . . . . . . . [<double>]. . . . . . . . . . -> get [set] random transition probability");

    QString learning_s(                       "\n    ----------------------Model Learning----------------------");
    QString episode_s(                          "    episode / e. . . . . . . . [<int>|clear,c] . . . . . . . -> record length <int> episode or clear data");
    QString optimize_s(                         "    optimize / o . . . . . . . [check, c]. . . . . . . . . . -> optimize CRF [check derivatives]");
    QString score_s(                            "    score. . . . . . . . . . . <int> . . . . . . . . . . . . -> score compound features with distance <int> by gradient");
    QString add_s(                              "    add. . . . . . . . . . . . <int> . . . . . . . . . . . . -> add <int> highest scored compound features to active (0 for all non-zero scored)");
    QString erase_s(                            "    erase. . . . . . . . . . . . . . . . . . . . . . . . . . -> erase features with zero weight");
    QString l1_s(                               "    l1 . . . . . . . . . . . . <double>. . . . . . . . . . . -> coefficient for L1 regularization");
    QString evaluate_s(                         "    evaluate . . . . . . . . . . . . . . . . . . . . . . . . -> evaluate features at current point");
    QString validate_s(                         "    validate / v . . . . . . . {crf,kmdp}[exact|mc <int>]. . -> validate CRF or k-MDP model using exact (default) or Monte Carlo (with <int> samples) computation of the KL-divergence");

    QString planning_s(                       "\n    -------------------------Planning--------------------------");
//    QString iterate_s(                          "    iterate / i. . . . . . . . [<int>|<double>,stop] . . . . -> run value iteration <int> times / until max diff small than <double> / stop running");
    QString discount_s(                         "    discount . . . . . . . . . [<double>]. . . . . . . . . . -> get [set] discount");
//    QString optimal_iteration_s(                "    optimal-iteration. . . . . . . . . . . . . . . . . . . . -> use known predictions for value iteration");
//    QString sparse_iteration_s(                 "    sparse-iteration . . . . . . . . . . . . . . . . . . . . -> use sparse model for value iteration");
//    QString kmdp_iteration_s(                   "    kmdp-iteration . . . . . . . . . . . . . . . . . . . . . -> use k-MDP model for value iteration");
    QString optimal_look_ahead_tree_s(          "    optimal-look-ahead-tree. . [<int>[<double>]] . . . . . . -> use known predictions for Look-Ahead-Tree [ depth [ threshold ] ]");
    QString sparse_look_ahead_tree_s(           "    sparse-look-ahead-tree . . [<int>[<double>]] . . . . . . -> use sparse model for Look-Ahead-Tree [ depth [ threshold ] ]");
    QString kmdp_look_ahead_tree_s(             "    kmdp-look-ahead-tree . . . [<int>[<double>]] . . . . . . -> use k-MDP model for Look-Ahead-Tree [ depth [ threshold ] ]");
    QString print_look_ahead_tree_s(            "    print-tree . . . . . . . . . . . . . . . . . . . . . . . -> print Look-Ahead-Tree");
    QString print_look_ahead_tree_statistics_s( "    print-tree-statistics. . . . . . . . . . . . . . . . . . -> print Look-Ahead-Tree statistics");
    QString max_tree_size_s(                    "    max-tree-size. . . . . . . <int> . . . . . . . . . . . . -> set maximum size of Look-Ahead-Tree (zero for infinite)");

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
//            ui._wConsoleOutput->appendPlainText( iterate_s );
            ui._wConsoleOutput->appendPlainText( discount_s );
//            ui._wConsoleOutput->appendPlainText( optimal_iteration_s );
//            ui._wConsoleOutput->appendPlainText( sparse_iteration_s );
//            ui._wConsoleOutput->appendPlainText( kmdp_iteration_s );
            ui._wConsoleOutput->appendPlainText( optimal_look_ahead_tree_s );
            ui._wConsoleOutput->appendPlainText( sparse_look_ahead_tree_s );
            ui._wConsoleOutput->appendPlainText( kmdp_look_ahead_tree_s );
            ui._wConsoleOutput->appendPlainText( print_look_ahead_tree_s );
            ui._wConsoleOutput->appendPlainText( print_look_ahead_tree_statistics_s );
            ui._wConsoleOutput->appendPlainText( max_tree_size_s );
        } else if(str_args[0]=="left" || str_args[0]=="l") { // left
            action_t action = action_t::LEFT;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="right" || str_args[0]=="r") { // right
            action_t action = action_t::RIGHT;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="up" || str_args[0]=="u") { // up
            action_t action = action_t::UP;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="down" || str_args[0]=="d") { // down
            action_t action = action_t::DOWN;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="stay" || str_args[0]=="s") { // stay
            action_t action = action_t::STAY;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
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
                ui._wConsoleOutput->appendPlainText( QString("    maze epsilon is %1").arg(maze.get_epsilon()) );
            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                maze.set_epsilon(double_args[1]);
            } else {
                ui._wConsoleOutput->appendPlainText( invalid_s );
                ui._wConsoleOutput->appendPlainText( epsilon_s );
            }
        } else if(str_args[0]=="discount") {
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText( QString("    discount is %1").arg(discount) );
            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                discount = double_args[1];
                look_ahead_search.set_discount(discount);
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
                if(str_args[2]=="mc") {
                    if( str_args.size()>3 && int_args_ok[3] && int_args[3]>0 ) {
                        probability_t model_l, maze_l;
                        probability_t kl = maze.validate_model<KMarkovCRF>(
                                crf,
                                crf.get_prediction_ptr(),
                                int_args[3],
                                &model_l,
                                &maze_l
                        );
                        ui._wConsoleOutput->appendPlainText(QString("    MC KL-Divergence = %1 (%2 samples)").arg(kl).arg(int_args[3]));
                        ui._wConsoleOutput->appendPlainText(QString("    Mean Likelihood: model = %1, maze = %2").arg(model_l).arg(maze_l));
                    } else {
                        ui._wConsoleOutput->appendPlainText( "    Please specify a valid sample size" );
                    }
                } else {
                    ui._wConsoleOutput->appendPlainText( invalid_s );
                    ui._wConsoleOutput->appendPlainText( validate_s );
                }
            } else if(str_args[1]=="kmdp") {
                ui._wConsoleOutput->appendPlainText( "    Sorry, not implemented" );
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
//            if(str_args.size()>1 && int_args_ok[1] && int_args[1]>0) {
//                tree_depth = int_args[1];
//                if(str_args.size()>2 && double_args_ok[2] && double_args[2]>=0) {
//                    probability_threshold = double_args[2];
//                } else {
//                    ui._wConsoleOutput->appendPlainText( "    Please specify a valid probability threshold" );
//                }
//            } else {
//                ui._wConsoleOutput->appendPlainText( "    Please specify a valid tree depth" );
//            }
        } else if(str_args[0]=="sparse-look-ahead-tree") {
            action_type = SPARSE_LOOK_AHEAD_TREE;
//            if(str_args.size()>1 && int_args_ok[1] && int_args[1]>0) {
//                tree_depth = int_args[1];
//                if(str_args.size()>2 && double_args_ok[2] && double_args[2]>=0) {
//                    probability_threshold = double_args[2];
//                } else {
//                    ui._wConsoleOutput->appendPlainText( "    Please specify a valid probability threshold" );
//                }
//            } else {
//                ui._wConsoleOutput->appendPlainText( "    Please specify a valid tree depth" );
//            }
        } else if(str_args[0]=="kmdp-look-ahead-tree") {
            action_type = KMDP_LOOK_AHEAD_TREE;
//            if(str_args.size()>1 && int_args_ok[1] && int_args[1]>0) {
//                tree_depth = int_args[1];
//                if(str_args.size()>2 && double_args_ok[2] && double_args[2]>=0) {
//                    probability_threshold = double_args[2];
//                } else {
//                    ui._wConsoleOutput->appendPlainText( "    Please specify a valid probability threshold" );
//                }
//            } else {
//                ui._wConsoleOutput->appendPlainText( "    Please specify a valid tree depth" );
//            }
        } else if(str_args[0]=="print-tree") { // print tree
            look_ahead_search.print_tree(true,true);
        } else if(str_args[0]=="print-tree-statistics") { // print tree statistics
            look_ahead_search.print_tree_statistics();
        } else if(str_args[0]=="max-tree-size") { // set tree size
            if(str_args.size()==1) {
                ui._wConsoleOutput->appendPlainText( QString( "    max tree size is %1" ).arg(max_tree_size) );
            } else if(int_args_ok[1] && int_args[1]>=0) {
                max_tree_size = int_args[1];
            } else {
                ui._wConsoleOutput->appendPlainText( "    Please specify a valid tree size" );
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

void TestMaze_II::back_in_history() {
    if(history_position>0) {
        --history_position;
    }
    ui._wConsoleInput->setText(console_history[history_position]);
}

void TestMaze_II::forward_in_history() {
    if(history_position==console_history.size()) {
        return;
    } else if(history_position<console_history.size()-1) {
        ++history_position;
    }
    ui._wConsoleInput->setText(console_history[history_position]);
}
