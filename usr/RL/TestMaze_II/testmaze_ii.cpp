#include "testmaze_ii.h"

#define DEBUG_LEVEL 1
#include "debug.h"

#define XDIM 2
#define YDIM 2

TestMaze_II::TestMaze_II(QWidget *parent)
    : QWidget(parent),
      maze(XDIM,YDIM,2,0.0),
      value_iteration_object(),
      crf(1,3,3,XDIM,YDIM,maze_t::NUMBER_OF_ACTIONS),
      random_action_timer(nullptr),
      value_iteration_timer(nullptr)
{
    // initialize UI
//    ui._wConsoleWidget->setTitleBarWidget(tmp);
    ui.setupUi(this);

    // focus on command line
    ui._wConsoleInput->setFocus();

    // seed random generator
    srand(time(nullptr));

    // set console welcome message
    ui._wConsoleOutput->setPlainText("    Please enter your commands (type 'help' for an overview)");

    // initialize random action timer
    random_action_timer = new QTimer(this);
    connect(random_action_timer, SIGNAL(timeout()), this, SLOT(random_action()));

    // initialize value iteration timer
    value_iteration_timer = new QTimer(this);
    connect(value_iteration_timer, SIGNAL(timeout()), this, SLOT(value_iteration()));

    // initialize transition model
    maze.initialize_transition_model(value_iteration_object);
    value_iteration_object.set_expected_reward(maze_t::state_t(0,0,XDIM),1);

    // initialize display
    maze.render_initialize(ui.graphicsView);

//    crf.check_derivative(10,10,1e-10,1e-3);
//    crf.optimize();
//    collect_episode(100);

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render()));
}

TestMaze_II::~TestMaze_II() {
    delete random_action_timer;
    delete value_iteration_timer;
}

void TestMaze_II::collect_episode(const int& length) {
    for(int idx=0; idx<length; ++idx) {
        maze_t::action_t action = (maze_t::action_t)(rand()%maze_t::NUMBER_OF_ACTIONS);
        maze_t::state_t state_to;
        double reward;
        maze.perform_transition(action,state_to,reward);
        crf.add_action_state_reward_tripel(action,state_to.idx(),reward);
    }
}

bool TestMaze_II::arg_int(const QString& string, const int& n, int& i) {
    QString arg = string.section(QRegExp("\\s+"),n,n);
    bool ok;
    i = arg.toInt(&ok);
    return ok;
}

bool TestMaze_II::arg_double(const QString& string, const int& n, double& d) {
    QString arg = string.section(QRegExp("\\s+"),n,n);
    bool ok;
    d = arg.toDouble(&ok);
    return ok;
}

bool TestMaze_II::arg_string(const QString& string, const int& n, QString& s) {
    s = string.section(QRegExp("\\s+"),n,n);
    return true;
}

void TestMaze_II::render() {
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::random_action() {
    maze.perform_transition(maze_t::action_t(rand()%maze_t::NUMBER_OF_ACTIONS));
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::value_iteration() {
    value_iteration_object.iterate();
    DEBUG_OUT(1,"State values:");
    for(int x_idx=0; x_idx<XDIM; ++x_idx) {
        for(int y_idx=0; y_idx<YDIM; ++y_idx) {
            DEBUG_OUT(1,"    (" << x_idx << "," << y_idx << ") --> " << value_iteration_object.get_value(maze_t::state_t(x_idx,y_idx,XDIM)));
        }
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
    QString random_s(   "    random  . . . .[<int>,stop]. . . . . . . -> start/stop random move");
    QString delay_s(    "    delay . . . . . <int>. . . . . . . . . . -> set reward delay");
    QString iterate_s(  "    iterate / i . .[<int>,stop]. . . . . . . -> start/stop value iteration");
    QString episode_s(  "    episode / e . .[<int>,clear] . . . . . . -> record length <int> episode or clear data");
    QString optimize_s( "    optimize / o   [check, c]. . . . . . . . -> optimize CRF [check derivatives]");
    QString epsilon_s(  "    epsilon . . . . <double> . . . . . . . . -> set exploration rate epsilon");

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
        ui._wConsoleOutput->appendPlainText( delay_s );
        ui._wConsoleOutput->appendPlainText( iterate_s );
        ui._wConsoleOutput->appendPlainText( episode_s );
        ui._wConsoleOutput->appendPlainText( optimize_s );
        ui._wConsoleOutput->appendPlainText( epsilon_s );
    } else if(input=="left" || input=="l") { // left
        maze.perform_transition(maze_t::LEFT);
        maze.render_update(ui.graphicsView);
    } else if(input=="right" || input=="r") { // right
        maze.perform_transition(maze_t::RIGHT);
        maze.render_update(ui.graphicsView);
    } else if(input=="up" || input=="u") { // up
        maze.perform_transition(maze_t::UP);
        maze.render_update(ui.graphicsView);
    } else if(input=="down" || input=="d") { // down
        maze.perform_transition(maze_t::DOWN);
        maze.render_update(ui.graphicsView);
    } else if(input=="stay" || input=="s") { // stay
        maze.perform_transition(maze_t::STAY);
        maze.render_update(ui.graphicsView);
    } else if(input.startsWith("random ") || input=="random") { // start/stop random actions
        QString s;
        int i;
        if(input=="random") {
            random_action();
        } else if(arg_string(input,1,s) && s=="stop") {
            random_action_timer->stop();
        } else if(arg_int(input,1,i) && i>=0){
            random_action_timer->stop();
            random_action_timer->start(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'random'. Expecting non-negative integer or 'stop', got '" + s + "'.");
        }
    } else if(input.startsWith("delay")) { // set time delay for rewards
        QString s;
        int i;
        if(input=="delay") {
            ui._wConsoleOutput->appendPlainText(delay_s);
        } else if(arg_int(input,1,i) && i>=0){
            maze.set_time_delay(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'delay'. Expecting non-negative integer, got '" + s + "'.");
        }
    } else if(input.startsWith("iterate ") || input.startsWith("i ") || input=="iterate" || input=="i") { // value iteration
        QString s;
        int i;
        if(input=="iterate" || input=="i") {
            value_iteration();
        } else if(arg_string(input,1,s) && s=="stop") {
            value_iteration_timer->stop();
        } else if(arg_int(input,1,i) && i>=0){
            value_iteration_timer->stop();
            value_iteration_timer->start(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'iterate'. Expecting non-negative integer or 'stop', got '" + s + "'.");
        }
    } else if(input.startsWith("episode ") || input.startsWith("e ") || input=="episode" || input=="e") { // record episode
        QString s;
        int i;
        if(input=="episode" || input=="e") {
            ui._wConsoleOutput->appendPlainText( episode_s );
        } else if(arg_string(input,1,s) && s=="clear") {
            crf.clear_data();
        } else if(arg_int(input,1,i) && i>=0){
            collect_episode(i);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'episode'. Expecting non-negative integer or 'clear', got '" + s + "'.");
        }
    } else if(input.startsWith("optimize ") || input.startsWith("o ") || input=="optimize" || input=="o") { // optimize CRF
        QString s1, s2;
        if(input=="optimize" || input=="o") {
            crf.optimize_model();
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
            ui._wConsoleOutput->appendPlainText("    " + QString::number(maze.get_epsilong()));
        } else if(arg_double(input,1,eps) && eps>=0 && eps<=1) {
            maze.set_epsilong(eps);
        } else {
            ui._wConsoleOutput->appendPlainText("    Invalid argument to 'epsilon'. Expecting double in [0,1], got '" + s + "'.");
        }
    } else if(input=="exit" || input=="quit" || input=="q") { // start/stop random actions
        QApplication::quit();
    } else {
        ui._wConsoleOutput->appendPlainText("    unknown command");
    }
}
