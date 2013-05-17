#include "testmaze_ii.h"
#include "Data.h"
#include "util.h"

#include <unistd.h> // for sleep()

#define DEBUG_LEVEL 1
#include "debug.h"

using util::arg_int;
using util::arg_double;
using util::arg_string;

#define TO_CONSOLE(x) { ui._wConsoleOutput->appendPlainText(x); }

TestMaze_II::TestMaze_II(QWidget *parent)
    : QWidget(parent),
      planner_type(OPTIMAL_PLANNER),
      maze(0.0),
      record(false), plot(false),
      current_instance(nullptr),
      random_timer(nullptr), action_timer(nullptr),
      console_history(1,"END OF HISTORY"),
      history_position(0),
      discount(0.7),
      l1_factor(0),
      utree(discount),
      look_ahead_search(discount),
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

    // install event filter
    MoveByKeys *moveByKeys = new MoveByKeys(this);
    ui.graphicsView->installEventFilter(moveByKeys);
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
        utree.add_action_state_reward_tripel(action,state_to,reward);
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
    if(record) {
        crf.add_action_state_reward_tripel(action,state_to,reward);
        utree.add_action_state_reward_tripel(action,state_to,reward);
    }
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::choose_action() {
    action_t action;
    switch(planner_type) {
    case OPTIMAL_PLANNER:
        look_ahead_search.clear_tree();
        look_ahead_search.build_tree<Maze>(
                current_instance,
                maze,
                maze.get_prediction_ptr(),
                max_tree_size
        );
        action = look_ahead_search.get_optimal_action();
        break;
    case SPARSE_PLANNER:
        look_ahead_search.clear_tree();
        look_ahead_search.build_tree<KMarkovCRF>(
                current_instance,
                crf,
                crf.get_prediction_ptr(),
                max_tree_size
        );
        action = look_ahead_search.get_optimal_action();
        break;
    case KMDP_PLANNER:
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
    case UTREE_PLANNER:
        look_ahead_search.clear_tree();
        look_ahead_search.build_tree<UTree>(
                current_instance,
                utree,
                utree.get_prediction_ptr(),
                max_tree_size
        );
        action = look_ahead_search.get_optimal_action();
        break;
    case UTREE_VALUE:
        action = utree.get_max_value_action(current_instance);
        break;
    default:
        action = action_t::STAY;
        DEBUG_OUT(0,"Error: undefined planner type --> choosing STAY");
        break;
    }
    state_t state_to;
    reward_t reward;
    maze.perform_transition(action,state_to,reward);
    update_current_instance(action,state_to,reward);
    if(record) {
        crf.add_action_state_reward_tripel(action,state_to,reward);
        utree.add_action_state_reward_tripel(action,state_to,reward);
    }
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
        TO_CONSOLE(input);
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
    QString headline_s(    "Available commands:\n    COMMAND . . . . ARGUMENTS. . . . . . . . . . . . . . . . . . .-> ACTION");

    QString general_s(                        "\n    -------------------------General---------------------------");
    QString help_s(                             "    help  / h. . . . . . . . . . . . . . . . . . . . . . . . . . .-> this help");
    QString exit_s(                             "    exit/quit/q. . . . . . . . . . . . . . . . . . . . . . . . . .-> quit application");
    QString set_s(                              "    set/unset. . . . . <string>. . . . . . . . . . . . . . . . . .-> set/unset options:");
    QString option_1_s(                         "                       record. . . . . . . . . . . . . . . . . . .-> start/stop recording movements");
    QString option_2_s(                         "                       plot. . . . . . . . . . . . . . . . . . . .-> write transitions into data file for plotting");
    QString option_3_s(                         "                       p(lanner) {o(ptimal)|s(parse)|u(tree)|uv} .-> set planner to use (uv = utree-value)");

    QString maze_s(                           "\n    ---------------------------Maze---------------------------");
    QString left_s(                             "    left  / l. . . . . . . . . . . . . . . . . . . . . . . . . . .-> move left");
    QString right_s(                            "    right / r. . . . . . . . . . . . . . . . . . . . . . . . . . .-> move right");
    QString up_s(                               "    up    / u. . . . . . . . . . . . . . . . . . . . . . . . . . .-> move up");
    QString down_s(                             "    down  / d. . . . . . . . . . . . . . . . . . . . . . . . . . .-> move down");
    QString stay_s(                             "    stay  / s. . . . . . . . . . . . . . . . . . . . . . . . . . .-> stay-action");
    QString move_s(                             "    move . . . . . . . [<int>|stop]. . . . . . . . . . . . . . . .-> start/stop moving using planner");
    QString random_s(                           "    random . . . . . . [<int>|stop]. . . . . . . . . . . . . . . .-> start/stop moving randomly");
    QString delay_s(                            "    delay. . . . . . . [<int>] . . . . . . . . . . . . . . . . . .-> get [set] reward delay");
    QString epsilon_s(                          "    epsilon. . . . . . [<double>]. . . . . . . . . . . . . . . . .-> get [set] random transition probability");

    QString learning_s(                       "\n    ----------------------Model Learning----------------------");
    QString episode_s(                          "    episode / e. . . . [<int>|clear,c] . . . . . . . . . . . . . .-> record length <int> episode or clear data");
    QString learning_crf_s(                     "    === CRF ===");
    QString optimize_s(                         "    optimize / o . . . [<int>|check, c]. . . . . . . . . . . . . .-> optimize CRF [max_iterations | check derivatives]");
    QString score_s(                            "    score. . . . . . . <int> . . . . . . . . . . . . . . . . . . .-> score compound features with distance <int> by gradient");
    QString add_s(                              "    add. . . . . . . . <int> . . . . . . . . . . . . . . . . . . .-> add <int> highest scored compound features to active (0 for all non-zero scored)");
    QString erase_s(                            "    erase. . . . . . . . . . . . . . . . . . . . . . . . . . . . .-> erase features with zero weight");
    QString l1_s(                               "    l1 . . . . . . . . <double>. . . . . . . . . . . . . . . . . .-> coefficient for L1 regularization");
    QString evaluate_s(                         "    evaluate . . . . . . . . . . . . . . . . . . . . . . . . . . .-> evaluate features at current point");
    QString validate_s(                         "    validate / v . . . {crf,kmdp}[exact|mc <int>]. . . . . . . . .-> validate CRF or k-MDP model using exact (default) or Monte Carlo (with <int> samples) computation of the KL-divergence");
    QString learning_utree_s(                   "    === UTree ===");
    QString expand_leaf_nodes_s(                "    expand / ex. . . . [<int>|<double] . . . . . . . . . . . . . .-> expand <int> leaf nodes / expand leaves until a score of <double> is reached");
    QString print_utree_s(                      "    print-utree. . . . . . . . . . . . . . . . . . . . . . . . . .-> print the current UTree");
    QString print_leaves_s(                     "    print-leaves . . . . . . . . . . . . . . . . . . . . . . . . .-> print leaves of the current UTree");
    QString clear_utree_s(                      "    clear-utree. . . . . . . . . . . . . . . . . . . . . . . . . .-> clear UTree");
    QString utree_q_iteration_s(                "    q-iteration / qi . .<int> <double> . . . . . . . . . . . . . .-> run <int> iterations of Q-Learning with alpha <double>");
    QString utree_value_iteration_s(            "    v-iteration / vi . .[<int>]. . . . . . . . . . . . . . . . . .-> run one [<int>] iteration(s) of Value-Iteration");
    QString utree_expansion_type_s(             "    ex-type / ext. . . .[u(tility)|s(tate)r(eward)]. . . . . . . .-> get/set expansion type for UTree");

    QString planning_s(                       "\n    -------------------------Planning--------------------------");
    QString discount_s(                         "    discount . . . . . [<double>]. . . . . . . . . . . . . . . . .-> get [set] discount");
    QString print_look_ahead_tree_s(            "    print-tree . . . . [g|graphic] . . . . . . . . . . . . . . . .-> print Look-Ahead-Tree [with graphical output]");
    QString max_tree_size_s(                    "    max-tree-size. . . <int> . . . . . . . . . . . . . . . . . . .-> set maximum size of Look-Ahead-Tree (zero for infinite)");

    set_s += "\n" + option_1_s;
    set_s += "\n" + option_2_s;
    set_s += "\n" + option_3_s;

    QString invalid_args_s( "    invalid arguments" );

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
            TO_CONSOLE( headline_s );
            // General
            TO_CONSOLE( general_s );
            TO_CONSOLE( help_s );
            TO_CONSOLE( exit_s );
            TO_CONSOLE( set_s );
            // Maze
            TO_CONSOLE( maze_s );
            TO_CONSOLE( left_s );
            TO_CONSOLE( right_s );
            TO_CONSOLE( up_s );
            TO_CONSOLE( down_s );
            TO_CONSOLE( stay_s );
            TO_CONSOLE( move_s );
            TO_CONSOLE( random_s );
            TO_CONSOLE( delay_s );
            TO_CONSOLE( epsilon_s );
            // Learning
            TO_CONSOLE( learning_s );
            TO_CONSOLE( episode_s );
            TO_CONSOLE( learning_crf_s ); // CRF
            TO_CONSOLE( optimize_s );
            TO_CONSOLE( score_s );
            TO_CONSOLE( add_s );
            TO_CONSOLE( erase_s );
            TO_CONSOLE( l1_s );
            TO_CONSOLE( evaluate_s );
            TO_CONSOLE( validate_s );
            TO_CONSOLE( learning_utree_s ); // UTree
            TO_CONSOLE( expand_leaf_nodes_s );
            TO_CONSOLE( print_utree_s );
            TO_CONSOLE( print_leaves_s );
            TO_CONSOLE( clear_utree_s );
            TO_CONSOLE( utree_q_iteration_s );
            TO_CONSOLE( utree_value_iteration_s );
            TO_CONSOLE( utree_expansion_type_s );
            // Planning
            TO_CONSOLE( planning_s );
            TO_CONSOLE( discount_s );
            TO_CONSOLE( print_look_ahead_tree_s );
            TO_CONSOLE( max_tree_size_s );
        } else if(str_args[0]=="left" || str_args[0]=="l") { // left
            action_t action = action_t::LEFT;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) {
                crf.add_action_state_reward_tripel(action,state_to,reward);
                utree.add_action_state_reward_tripel(action,state_to,reward);
            }
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="right" || str_args[0]=="r") { // right
            action_t action = action_t::RIGHT;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) {
                crf.add_action_state_reward_tripel(action,state_to,reward);
                utree.add_action_state_reward_tripel(action,state_to,reward);
            }
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="up" || str_args[0]=="u") { // up
            action_t action = action_t::UP;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) {
                crf.add_action_state_reward_tripel(action,state_to,reward);
                utree.add_action_state_reward_tripel(action,state_to,reward);
            }
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="down" || str_args[0]=="d") { // down
            action_t action = action_t::DOWN;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) {
                crf.add_action_state_reward_tripel(action,state_to,reward);
                utree.add_action_state_reward_tripel(action,state_to,reward);
            }
            maze.render_update(ui.graphicsView);
        } else if(str_args[0]=="stay" || str_args[0]=="s") { // stay
            action_t action = action_t::STAY;
            state_t state_to;
            reward_t reward;
            maze.perform_transition(action,state_to,reward);
            update_current_instance(action,state_to,reward);
            if(record) {
                crf.add_action_state_reward_tripel(action,state_to,reward);
                utree.add_action_state_reward_tripel(action,state_to,reward);
            }
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
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( move_s );
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
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( random_s );
            }
        } else if(str_args[0]=="delay") { // set time delay for rewards
            if(str_args.size()==1) {
                TO_CONSOLE("    " + QString::number(maze.get_time_delay()));
            } else if(int_args_ok[1] && int_args[1]>=0){
                maze.set_time_delay(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( delay_s );
            }
        } else if(str_args[0]=="episode" || str_args[0]=="e") { // record episode
            if(str_args.size()==1) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( episode_s );
            } else if( str_args[1]=="clear" || str_args[1]=="c" ) {
                crf.clear_data();
                utree.clear_data();
            } else if(int_args_ok[1] && int_args[1]>=0){
                collect_episode(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( episode_s );
            }
        } else if(str_args[0]=="optimize" || str_args[0]=="o") { // optimize CRF
            if(str_args.size()==1 || int_args_ok[1] ) {
                if(int_args_ok[1]) {
                    crf.optimize_model(l1_factor, int_args[1]);
                } else {
                    crf.optimize_model(l1_factor, 0);
                }
            } else if(str_args[1]=="check" || str_args[1]=="c") {
                crf.check_derivatives(3,10,1e-6,1e-3);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( optimize_s );
            }
        } else if(str_args[0]=="epsilon") {
            if(str_args.size()==1) {
                TO_CONSOLE( QString("    maze epsilon is %1").arg(maze.get_epsilon()) );
            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                maze.set_epsilon(double_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( epsilon_s );
            }
        } else if(str_args[0]=="expand" || str_args[0]=="ex") {
            if(str_args.size()==1) {
                double score = utree.expand_leaf_node();
                TO_CONSOLE( QString("    Score was %1").arg(score) );
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                double score = 0;
                for(int i=0; i<int_args[1]; ++i) {
                    score = utree.expand_leaf_node();
                    // todo WHAT??!! Instances of root node are
                    // inserted twice into new node without usleep().
                    usleep(1);
                }
                TO_CONSOLE( QString("    Last score was %1").arg(score) );
            } else if(double_args_ok[1]) {
                while( double_args[1] <= utree.expand_leaf_node(double_args[1]) ) {}
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( expand_leaf_nodes_s );
            }
        } else if(str_args[0]=="print-utree") {
            utree.print_tree();
        } else if(str_args[0]=="print-leaves") {
            utree.print_leaves();
        } else if(str_args[0]=="clear-utree") {
            utree.clear_tree();
        } else if(str_args[0]=="q-iteration" || str_args[0]=="qi") {
            if(str_args.size()==3 &&
               int_args_ok[1] && int_args[1]>0 &&
               double_args_ok[2] && double_args[2]>=0 && double_args[2]<=1)
            {
                double max_diff;
                repeat(int_args[1]) {
                    max_diff = utree.q_iteration(double_args[2]);
                }
                TO_CONSOLE( QString(    "run %1 iteration(s) with alpha=%2, last maximum update was %3").arg(int_args[1]).arg(double_args[2]).arg(max_diff) );
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( utree_q_iteration_s );
            }
        } else if(str_args[0]=="v-iteration" || str_args[0]=="vi") {
            if( str_args.size()==1 || (str_args.size()>1 && int_args_ok[1] && int_args[1]>0) ) {
                double max_diff;
                int rep = str_args.size()==1 ? 1 : int_args[1];
                repeat(rep) {
                    max_diff = utree.value_iteration();
                }
                TO_CONSOLE( QString(    "run %1 iteration(s), last maximum update was %2").arg(rep).arg(max_diff) );
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( utree_value_iteration_s );
            }
        } else if(str_args[0]=="ex-type" || str_args[0]=="ext") {
            if(str_args.size()==1) {
                switch(utree.get_expansion_type()) {
                case UTree::UTILITY_EXPANSION:
                    TO_CONSOLE("    expansion type: UTILITY_EXPANSION");
                    break;
                case UTree::STATE_REWARD_EXPANSION:
                    TO_CONSOLE("    expansion type: STATE_REWARD_EXPANSION");
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }
            } else if(str_args[1]=="utility" || str_args[1]=="u") {
                utree.set_expansion_type(UTree::UTILITY_EXPANSION);
            } else if(str_args[1]=="statereward" ||str_args[1]=="sr") {
                utree.set_expansion_type(UTree::STATE_REWARD_EXPANSION);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( utree_expansion_type_s );
            }
        } else if(str_args[0]=="discount") {
            if(str_args.size()==1) {
                TO_CONSOLE( QString("    discount is %1").arg(discount) );
            } else if(double_args_ok[1] && double_args[1]>=0 && double_args[1]<=1) {
                discount = double_args[1];
                look_ahead_search.set_discount(discount);
                utree.set_discount(discount);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( discount_s );
            }
        } else if(str_args[0]=="evaluate") {
            crf.evaluate_features();
        } else if(str_args[0]=="validate" || str_args[0]=="v") {
            if(str_args.size()==1 ) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( validate_s );
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
                        TO_CONSOLE(QString("    MC KL-Divergence = %1 (%2 samples)").arg(kl).arg(int_args[3]));
                        TO_CONSOLE(QString("    Mean Likelihood: model = %1, maze = %2").arg(model_l).arg(maze_l));
                    } else {
                        TO_CONSOLE( "    Please specify a valid sample size" );
                    }
                } else {
                    TO_CONSOLE( invalid_args_s );
                    TO_CONSOLE( validate_s );
                }
            } else if(str_args[1]=="kmdp") {
                TO_CONSOLE( "    Sorry, not implemented" );
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( validate_s );
            }
        } else if(str_args[0]=="l1") {
            if(str_args.size()==1) {
                TO_CONSOLE(QString("    %1").arg(l1_factor));
            } else if(double_args_ok[1] && double_args[1]>=0) {
                l1_factor = double_args[1];
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( l1_s );
            }
        } else if(str_args[0]=="score") {
            if(str_args.size()==1) {
                TO_CONSOLE(score_s);
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                crf.score_features_by_gradient(int_args[1]);
                crf.sort_scored_features();
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( score_s );
            }
        } else if(str_args[0]=="add") {
            if(str_args.size()==1) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( add_s );
            } else if(int_args_ok[1] && int_args[1]>=0 ) {
                crf.add_compound_features_to_active(int_args[1]);
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( add_s );
            }
        } else if(str_args[0]=="erase") {
            crf.erase_zero_features();
        } else if(str_args[0]=="exit" || str_args[0]=="quit" || str_args[0]=="q") { // quit application
            QApplication::quit();
        } else if(str_args[0]=="print-tree") { // print tree
            if(str_args.size()>1) {
                if(str_args[1]=="g" || str_args[1]=="graphic") {
                    look_ahead_search.print_tree(true,true);
                    look_ahead_search.print_tree_statistics();
                } else {
                    TO_CONSOLE( invalid_args_s );
                    TO_CONSOLE( print_look_ahead_tree_s );
                }
            } else {
                look_ahead_search.print_tree_statistics();
            }
        } else if(str_args[0]=="max-tree-size") { // set tree size
            if(str_args.size()==1) {
                TO_CONSOLE( QString( "    max tree size is %1" ).arg(max_tree_size) );
            } else if(int_args_ok[1] && int_args[1]>=0) {
                max_tree_size = int_args[1];
            } else {
                TO_CONSOLE( "    Please specify a valid tree size" );
            }
        } else if(str_args[0]=="set" || str_args[0]=="unset") { // set option
            if(str_args.size()==1) {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( set_s );
            } else if(str_args[1]=="record") {
                    record = str_args[0]=="set";
                    if(record) {
                        TO_CONSOLE( "    record on" );
                    } else {
                        TO_CONSOLE( "    record off" );
                    }
            } else if(str_args[1]=="plot") {
                plot = str_args[0]=="set";
                if(plot) {
                    // open plot file
                    plot_file.open("plot_file.txt");
                    plot_file << "# action state reward" << std::endl;
                    TO_CONSOLE( "    plot on" );
                } else {
                    // close plot file
                    plot_file.close();
                    TO_CONSOLE( "    plot off" );
                }
            } else if( (str_args[1]=="p" || str_args[1]=="planner") && str_args.size()>2) {
                if(str_args[0]=="unset") {
                    TO_CONSOLE( "    set different planner to unset current" );
                } else if(str_args[2]=="optimal" || str_args[2]=="o") {
                    planner_type = OPTIMAL_PLANNER;
                    TO_CONSOLE( "    using optimal planner" );
                } else if(str_args[2]=="sparse" || str_args[2]=="s") {
                    planner_type = SPARSE_PLANNER;
                    TO_CONSOLE( "    using sparse planner" );
                } else if(str_args[2]=="utree" || str_args[2]=="u") {
                    planner_type = UTREE_PLANNER;
                    TO_CONSOLE( "    using UTree planner" );
                } else if(str_args[2]=="uv") {
                    planner_type = UTREE_VALUE;
                    TO_CONSOLE( "    using UTree-value for action selection" );
                }
            } else {
                TO_CONSOLE( invalid_args_s );
                TO_CONSOLE( set_s );
            }
        } else {
            TO_CONSOLE("    unknown command");
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

bool MoveByKeys::eventFilter(QObject *obj, QEvent *event)
{
    // process some key events
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
	switch(keyEvent->key()) {
	case Qt::Key_Left: // left
            maze->process_console_input(QString('l'), true);
            return true;
	case Qt::Key_Right: // right
            maze->process_console_input(QString('r'), true);
            return true;
	case Qt::Key_Up: // up
            maze->process_console_input(QString('u'), true);
            return true;
	case Qt::Key_Down: // down
            maze->process_console_input(QString('d'), true);
            return true;
	case Qt::Key_Space: // space
            maze->process_console_input(QString('s'), true);
            return true;
	case Qt::Key_Return:
	case Qt::Key_Enter:
	default:
            break;
	}
    }

    // standard event processing
    return QObject::eventFilter(obj, event);
}
