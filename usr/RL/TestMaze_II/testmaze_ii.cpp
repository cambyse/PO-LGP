#include "testmaze_ii.h"

#define DEBUG_LEVEL 1
#include "debug.h"

#define XDIM 2
#define YDIM 2

TestMaze_II::TestMaze_II(QWidget *parent)
    : QWidget(parent),
      maze(XDIM,YDIM,1,0.0),
      value_iteration_object(),
      crf(0,XDIM,YDIM,maze_t::NUMBER_OF_ACTIONS),
      random_action_timer(NULL),
      value_iteration_timer(NULL)
{
    // initialize UI
    ui.setupUi(this);

    // focus on command line
    ui._wConsoleInput->setFocus();

    // seed random generator
    srand(time(NULL));

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
    collect_episode(100);

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

int TestMaze_II::arg_int(QString& string, const int& n, bool * ok) {
    QString arg = string.section(QRegExp("\\s+"),n,n);
    int ret = arg.toInt(ok);
    if(!ok) ret = 0;
    return ret;
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

    // process input
    if(input=="help") { // help
        const char * help_text = "    Available commands:\n\
        help              -> this help\n\
        left  / l         -> move left\n\
        right / r         -> move right\n\
        up    / u         -> move up\n\
        down  / d         -> move down\n\
        stay  / s         -> stay-action\n\
        random [int,stop] -> start/stop random move";
        ui._wConsoleOutput->appendPlainText(help_text);
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
    } else if(input.startsWith("random")) { // start/stop random actions
        bool invalid_argument = true;
        QString arg = input.remove("random");
        arg.remove(" ");
        if(arg=="stop") {
            invalid_argument = false;
            random_action_timer->stop();
        } else {
            bool ok;
            int delay = arg.toInt(&ok);
            if( ok && delay>=0 ) {
                invalid_argument = false;
                random_action_timer->stop();
                random_action_timer->start(delay);
            }
        }
        if(invalid_argument) {
            QString error_text;
            error_text.append("    Invalid argument to 'random'. Expecting non-negative integer or 'stop', got '")
            .append(arg)
            .append("'.");
            ui._wConsoleOutput->appendPlainText(error_text);
        }
    } else if(input.startsWith("delay")) { // set time delay for rewards
        bool invalid_argument = true;
        QString arg = input.remove("delay");
        arg.remove(" ");
        bool ok;
        int delay = arg.toInt(&ok);
        if( ok && delay>=0 ) {
            invalid_argument = false;
            maze.set_time_delay(delay);
        }
        if(invalid_argument) {
            QString error_text;
            error_text.append("    Invalid argument to 'delay'. Expecting non-negative integer, got '")
            .append(arg)
            .append("'.");
            ui._wConsoleOutput->appendPlainText(error_text);
        }
    } else if(input.startsWith("iterate")) { // value iteration
        QString arg = input.remove("iterate");
        arg.remove(" ");
        bool invalid_argument = true;
        if(arg=="stop") {
            invalid_argument = false;
            value_iteration_timer->stop();
        } else {
            bool ok;
            int delay = arg.toInt(&ok);
            if( ok && delay>=0 ) {
                invalid_argument = false;
                value_iteration_timer->stop();
                value_iteration_timer->start(delay);
            }
        }
        if(invalid_argument) {
            QString error_text;
            error_text.append("    Invalid argument to 'iterate'. Expecting non-negative integer got '")
                            .append(arg)
                            .append("'.");
            ui._wConsoleOutput->appendPlainText(error_text);
        }
    } else if(input=="exit" || input=="quit") { // start/stop random actions
        QApplication::quit();
    } else if(input.startsWith("arg")) { // parse arguments
        int counter = 0;
        input.remove("arg");
        bool ok = true;
        while(ok) {
            int arg = arg_int(input,counter+1,&ok);
            if(ok) {
                ++counter;
                DEBUG_OUT(0,"    argument nr. " << counter << ": " << arg);
            }
        }
        DEBUG_OUT(0,"    Parsed " << counter << " arguments." );
    } else {
        ui._wConsoleOutput->appendPlainText("    unknown command");
    }
}
