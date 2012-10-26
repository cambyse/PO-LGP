#include "testmaze_ii.h"
#include "debug.h"

TestMaze_II::TestMaze_II(QWidget *parent)
    : QWidget(parent), maze(2,2), random_action_timer(NULL)
{
    // initialize UI
    ui.setupUi(this);

    // seed random generator
    srand(time(NULL));

    // set console welcome message
    ui._wConsoleOutput->setPlainText("    Please enter your commands (type 'help' for an overview)");

    // initialize random action timer
    random_action_timer = new QTimer(this);
    connect(random_action_timer, SIGNAL(timeout()), this, SLOT(random_action()));

    // focus on command line
    ui._wConsoleInput->setFocus();

    // initialize display
    maze.render_initialize(ui.graphicsView);

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render()));
}

TestMaze_II::~TestMaze_II() {
    delete random_action_timer;
}

void TestMaze_II::render() {
    maze.render_update(ui.graphicsView);
}

void TestMaze_II::random_action() {
    maze.perform_transition(maze_t::action_t(rand()%maze_t::NUMBER_OF_ACTIONS));
    maze.render_update(ui.graphicsView);
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
            error_text.append("    Invalid argument to 'delay'. Expecting non-negative integer.")
            .append(arg)
            .append("'.");
            ui._wConsoleOutput->appendPlainText(error_text);
        }
    } else if(input=="exit" || input=="quit") { // start/stop random actions
        QApplication::quit();
    } else {
        ui._wConsoleOutput->appendPlainText("    unknown command");
    }
}
