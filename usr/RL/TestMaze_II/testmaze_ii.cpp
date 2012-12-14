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
      action_type(RANDOM),
      iteration_type(INT), iteration_number(0), iteration_counter(0), iteration_threshold(1),
      maze(0.0),
      q_iteration_object(),
      record(false), plot(false),
      l1_factor(0),
      current_k_mdp_state(),
      action_timer(nullptr),
      value_iteration_timer(nullptr)
{
    // initialize UI
    ui.setupUi(this);

    // focus on command line
    ui._wConsoleInput->setFocus();

    // set console welcome message
    ui._wConsoleOutput->setPlainText("    Please enter your commands (type 'help' for an overview)");

    // initialize random action timer
    action_timer = new QTimer(this);
    connect(action_timer, SIGNAL(timeout()), this, SLOT(choose_action()));

    // initialize value iteration timer
    value_iteration_timer = new QTimer(this);
    connect(value_iteration_timer, SIGNAL(timeout()), this, SLOT(value_iteration()));

    // initialize transition model
    maze.initialize_predictions(q_iteration_object);

    // initialize display
    maze.render_initialize(ui.graphicsView);

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render()));

}

TestMaze_II::~TestMaze_II() {
    delete action_timer;
    delete value_iteration_timer;
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

void TestMaze_II::choose_action() {
    action_t action;
    switch(action_type) {
    case RANDOM:
        action = (action_t)(rand()%Data::action_n);
        break;
    case OPTIMAL:
        action = q_iteration_object.optimal_action(current_k_mdp_state.get_k_mdp_state());
        break;
    default:
        DEBUG_OUT(0,"Error: undefined action type");
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
    reward_t max_value_diff = q_iteration_object.iterate();
    switch(iteration_type) {
    case INT:
        ++iteration_counter;
        if(iteration_counter>=iteration_number) {
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
}

void TestMaze_II::process_console_input() {
    QString input = ui._wConsoleInput->text();
    ui._wConsoleInput->setText("");
    ui._wConsoleOutput->appendPlainText(input);

    QString headline_s( "    COMMAND . . .  ARGUMENTS . . . . . . . . -> ACTION\n");
    QString help_s(     "    help  / h. . . . . . . . . . . . . . . . -> this help");
    QString left_s(     "    left  / l. . . . . . . . . . . . . . . . -> move left");
    QString right_s(    "    right / r. . . . . . . . . . . . . . . . -> move right");
    QString up_s(       "    up    / u. . . . . . . . . . . . . . . . -> move up");
    QString down_s(     "    down  / d. . . . . . . . . . . . . . . . -> move down");
    QString stay_s(     "    stay  / s. . . . . . . . . . . . . . . . -> stay-action");
    QString random_s(   "    random  . . . .[<int>|stop]. . . . . . . -> start/stop random moves");
    QString optimal_s(  "    optimal . . . .[<int>|stop]. . . . . . . -> start/stop optimal moves");
    QString delay_s(    "    delay . . . . .[<int>] . . . . . . . . . -> get [set] reward delay");
    QString iterate_s(  "    iterate / i . .[<int>|<double>,stop] . . -> run value iteration <int> times / until max diff small than <double> / stop running");
    QString episode_s(  "    episode / e . .[<int>|clear,c] . . . . . -> record length <int> episode or clear data");
    QString optimize_s( "    optimize / o   [check, c]. . . . . . . . -> optimize CRF [check derivatives]");
    QString epsilon_s(  "    epsilon . . . .[<double>]. . . . . . . . -> get [set] exploration rate epsilon");
    QString discount_s( "    discount. . . .[<double>]. . . . . . . . -> get [set] discount rate for value iteration");
    QString evaluate_s( "    evaluate . . . . . . . . . . . . . . . . -> evaluate features at current point");
    QString l1_s(       "    l1 . . . . . . . <double>. . . . . . . . -> coefficient for L1 regularization");
    QString score_s(    "    score. . . . . .<int>. . . . . . . . . . -> score compound features with distance <int> by gradient");
    QString add_s(      "    add. . . . . . .<int>. . . . . . . . . . -> add <int> highest scored compound features to active (give 0 for all non-zero scored)");
    QString erase_s(    "    erase. . . . . . . . . . . . . . . . . . -> erase features with zero weight");
    QString exit_s(     "    exit/quit/q. . . . . . . . . . . . . . . -> quit application");
    QString set_s(      "    set/unset. . . . . . <string>. . . . . . -> set/unset option:");
    QString option_1_s( "                   optimaliteration. . . . . -> use known predictions for value iteration");
    QString option_2_s( "                   sparseiteration . . . . . -> use sparse model for value iteration");
    QString option_3_s( "                   kmdpiteration . . . . . . -> use k-MDP model for value iteration");
    QString option_4_s( "                   record. . . . . . . . . . -> start/stop recording movements");
    QString option_5_s( "                   plot. . . . . . . . . . . -> write transitions into data file for plotting");

    set_s += "\n" + option_1_s;
    set_s += "\n" + option_2_s;
    set_s += "\n" + option_3_s;
    set_s += "\n" + option_4_s;
    set_s += "\n" + option_5_s;

    // process input
    if(input=="help" || input=="h") { // help
        ui._wConsoleOutput->appendPlainText("    Available commands:\n");
        ui._wConsoleOutput->appendPlainText( headline_s );
        ui._wConsoleOutput->appendPlainText( help_s );
        ui._wConsoleOutput->appendPlainText( left_s );
        ui._wConsoleOutput->appendPlainText( right_s );
        ui._wConsoleOutput->appendPlainText( up_s );
        ui._wConsoleOutput->appendPlainText( down_s );
        ui._wConsoleOutput->appendPlainText( stay_s );
        ui._wConsoleOutput->appendPlainText( random_s );
        ui._wConsoleOutput->appendPlainText( optimal_s );
        ui._wConsoleOutput->appendPlainText( delay_s );
        ui._wConsoleOutput->appendPlainText( iterate_s );
        ui._wConsoleOutput->appendPlainText( episode_s );
        ui._wConsoleOutput->appendPlainText( optimize_s );
        ui._wConsoleOutput->appendPlainText( epsilon_s );
        ui._wConsoleOutput->appendPlainText( discount_s );
        ui._wConsoleOutput->appendPlainText( evaluate_s );
        ui._wConsoleOutput->appendPlainText( l1_s );
        ui._wConsoleOutput->appendPlainText( score_s );
        ui._wConsoleOutput->appendPlainText( add_s );
        ui._wConsoleOutput->appendPlainText( erase_s );
        ui._wConsoleOutput->appendPlainText( exit_s );
        ui._wConsoleOutput->appendPlainText( set_s );
    } else if(input=="left" || input=="l") { // left
        action_t action = Data::LEFT;
        state_t state_to;
        reward_t reward;
        maze.perform_transition(action,state_to,reward);
        update_current_k_mdp_state(action,state_to,reward);
        if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
        maze.render_update(ui.graphicsView);
    } else if(input=="right" || input=="r") { // right
        action_t action = Data::RIGHT;
        state_t state_to;
        reward_t reward;
        maze.perform_transition(action,state_to,reward);
        update_current_k_mdp_state(action,state_to,reward);
        if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
        maze.render_update(ui.graphicsView);
    } else if(input=="up" || input=="u") { // up
        action_t action = Data::UP;
        state_t state_to;
        reward_t reward;
        maze.perform_transition(action,state_to,reward);
        update_current_k_mdp_state(action,state_to,reward);
        if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
        maze.render_update(ui.graphicsView);
    } else if(input=="down" || input=="d") { // down
        action_t action = Data::DOWN;
        state_t state_to;
        reward_t reward;
        maze.perform_transition(action,state_to,reward);
        update_current_k_mdp_state(action,state_to,reward);
        if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
        maze.render_update(ui.graphicsView);
    } else if(input=="stay" || input=="s") { // stay
        action_t action = Data::STAY;
        state_t state_to;
        reward_t reward;
        maze.perform_transition(action,state_to,reward);
        update_current_k_mdp_state(action,state_to,reward);
        if(record) crf.add_action_state_reward_tripel(action,state_to,reward);
        maze.render_update(ui.graphicsView);
    } else if(input.startsWith("random ") || input=="random") { // start/stop random actions
        QString s;
        int i;
        if(input=="random") {
            action_type = RANDOM;
            choose_action();
        } else if(arg_string(input,1,s) && s=="stop") {
            action_timer->stop();
        } else if(arg_int(input,1,i) && i>=0){
            action_timer->stop();
            action_type = RANDOM;
            action_timer->start(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'random'. Expecting non-negative integer or 'stop', got '" + s + "'.");
        }
    } else if(input.startsWith("optimal ") || input=="optimal") { // start/stop optimal actions
        QString s;
        int i;
        if(input=="optimal") {
            action_type = OPTIMAL;
            choose_action();
        } else if(arg_string(input,1,s) && s=="stop") {
            action_timer->stop();
        } else if(arg_int(input,1,i) && i>=0){
            action_timer->stop();
            action_type = OPTIMAL;
            action_timer->start(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'optimal'. Expecting non-negative integer or 'stop', got '" + s + "'.");
        }
    } else if(input.startsWith("delay ") || input=="delay") { // set time delay for rewards
        QString s;
        int i;
        if(input=="delay") {
            ui._wConsoleOutput->appendPlainText("    " + QString::number(maze.get_time_delay()));
        } else if(arg_int(input,1,i) && i>=0){
            maze.set_time_delay(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'delay'. Expecting non-negative integer, got '" + s + "'.");
        }
    } else if(input.startsWith("iterate ") || input.startsWith("i ") || input=="iterate" || input=="i") { // value iteration
        QString s;
        int i;
        double d;
        if(input=="iterate" || input=="i") {
            value_iteration();
        } else if(arg_string(input,1,s) && s=="stop") {
            value_iteration_timer->stop();
        } else if(arg_int(input,1,i) && i>=0){
            value_iteration_timer->stop();
            iteration_type = INT;
            iteration_number = i;
            iteration_counter = 0;
            value_iteration_timer->start();
        } else if(arg_double(input,1,d) && d>=0){
            value_iteration_timer->stop();
            iteration_type = DOUBLE;
            iteration_threshold = d;
            value_iteration_timer->start();
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'iterate'. Expecting non-negative integer or 'stop', got '" + s + "'.");
        }
    } else if(input.startsWith("episode ") || input.startsWith("e ") || input=="episode" || input=="e") { // record episode
        QString s;
        int i;
        if(input=="episode" || input=="e") {
            ui._wConsoleOutput->appendPlainText( episode_s );
        } else if(arg_string(input,1,s) && ( s=="clear" || s=="c" ) ) {
            crf.clear_data();
        } else if(arg_int(input,1,i) && i>=0){
            collect_episode(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'episode'. Expecting non-negative integer or 'clear', got '" + s + "'.");
        }
    } else if(input.startsWith("optimize ") || input.startsWith("o ") || input=="optimize" || input=="o") { // optimize CRF
        QString s1, s2;
        if(input=="optimize" || input=="o") {
            crf.optimize_model(l1_factor);
        } else if(arg_string(input,1,s1) && ( s1=="check" || s1=="c") ) {
                crf.check_derivatives(3,10,1e-6,1e-3);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'optimize'. Expecting no argument, [check [reward,r,state,s], c [reward,r,state,s], reward, r, state, s], got '" + s1 + " " + s2 + "'.");
        }
    } else if(input.startsWith("epsilon ") || input=="epsilon") {
        QString s;
        arg_string(input,1,s);
        double eps;
        if(input=="epsilon") {
            ui._wConsoleOutput->appendPlainText("    " + QString::number(maze.get_epsilon()));
        } else if(arg_double(input,1,eps) && eps>=0 && eps<=1) {
            maze.set_epsilon(eps);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'epsilon'. Expecting double in [0,1], got '" + s + "'.");
        }
    } else if(input.startsWith("discount ") || input=="discount") {
        QString s;
        arg_string(input,1,s);
        double disc;
        if(input=="discount") {
            ui._wConsoleOutput->appendPlainText("    " + QString::number(q_iteration_object.get_discount()));
        } else if(arg_double(input,1,disc) && disc>=0 && disc<=1) {
            q_iteration_object.set_discount(disc);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'discount'. Expecting double in [0,1], got '" + s + "'.");
        }
    } else if(input=="evaluate") {
        crf.evaluate_features();
    } else if(input.startsWith("l1 ") || input=="l1") {
        QString s;
        double c;
        if(input=="l1") {
            ui._wConsoleOutput->appendPlainText(QString("    %1").arg(l1_factor));
        } else if(arg_double(input,1,c) && c>=0) {
            l1_factor = c;
        }
    } else if(input.startsWith("score ") || input=="score") {
        int n;
        if(input=="score") {
            ui._wConsoleOutput->appendPlainText(score_s);
        } else if(arg_int(input,1,n) && n>=0 ) {
            crf.score_features_by_gradient(n);
            crf.sort_scored_features();
        } else {
            ui._wConsoleOutput->appendPlainText(score_s);
        }
    } else if(input.startsWith("add ") || input=="add") {
        int n;
        if(input=="add") {
            ui._wConsoleOutput->appendPlainText(add_s);
        } else if(arg_int(input,1,n) && n>=0 ) {
            crf.add_compound_features_to_active(n);
        } else {
            ui._wConsoleOutput->appendPlainText(add_s);
        }
    } else if(input=="erase") {
        crf.erase_zero_features();
    } else if(input=="exit" || input=="quit" || input=="q") { // quit application
        QApplication::quit();
    } else if(input.startsWith("set ") || input=="set" || input.startsWith("unset ") || input=="unset") { // quit application
        QString s_set_unset;
        if(!arg_string(input,0,s_set_unset) || (s_set_unset!="set" && s_set_unset!="unset") ) {
            DEBUG_OUT(0,"Error: something went wrong parsing the 'set' command");
        }
        QString s_arg;
        if(input=="set" || input=="unset") {
            ui._wConsoleOutput->appendPlainText( set_s );
        } else if(arg_string(input,1,s_arg)) {
            if(s_arg=="optimaliteration") {
                if(s_set_unset=="unset") {
                    ui._wConsoleOutput->appendPlainText("    use 'set [kmdpiteration|sparseiteration]' to unset 'optimaliteration'");
                } else {
                    q_iteration_object.clear();
                    maze.initialize_predictions(q_iteration_object);
                    ui._wConsoleOutput->appendPlainText("    initialized prediction matrix with true values");
                }
            } else if(s_arg=="sparseiteration") {
                if(s_set_unset=="unset") {
                    ui._wConsoleOutput->appendPlainText("    use 'set [optimaliteration|kmdpiteration]' to unset 'sparseiteration'");
                } else {
                    q_iteration_object.clear();
                    crf.initialize_sparse_predictions(q_iteration_object);
                    ui._wConsoleOutput->appendPlainText("    initialized prediction matrix values from sparse model");
                }
            } else if(s_arg=="kmdpiteration") {
                if(s_set_unset=="unset") {
                    ui._wConsoleOutput->appendPlainText("    use 'set [optimaliteration|sparseiteration]' to unset 'kmdpiteration'");
                } else {
                    q_iteration_object.clear();
                    crf.initialize_kmdp_predictions(q_iteration_object);
                    ui._wConsoleOutput->appendPlainText("    initialized prediction matrix values with relative frequencies");
                }
            } else if(s_arg=="record") {
                record = s_set_unset=="set";
                if(record) {
                    ui._wConsoleOutput->appendPlainText( "    record on" );
                } else {
                    ui._wConsoleOutput->appendPlainText( "    record off" );
                }
            } else if(s_arg=="plot") {
                plot = s_set_unset=="set";
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
                ui._wConsoleOutput->appendPlainText( "    unknown option" );
                ui._wConsoleOutput->appendPlainText( set_s );

            }
        } else {
            ui._wConsoleOutput->appendPlainText( "    unknown option (must be string)" );
            ui._wConsoleOutput->appendPlainText( set_s );
        }
    } else {
        ui._wConsoleOutput->appendPlainText("    unknown command");
    }
}
