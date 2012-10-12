#include "testmaze_ii.h"

TestMaze_II::TestMaze_II(QWidget *parent)
    : QWidget(parent), maze(5,5)
{
    // initialize UI
    ui.setupUi(this);

    // seed random generator
    srand(time(NULL));

    // set timer for random actions
    random_action_timer = new QTimer(this);
    connect(random_action_timer, SIGNAL(timeout()), this, SLOT(random_action()));
    random_action_timer->start(200);

    // initiate delayed render action
    QTimer::singleShot(0, this, SLOT(render()));
}

TestMaze_II::~TestMaze_II()
{

}

void TestMaze_II::render() {
    maze.render(ui.graphicsView);
}

void TestMaze_II::random_action() {
    maze.perform_transition(Maze::action(rand()%Maze::NUMBER_OF_ACTIONS));
    maze.render(ui.graphicsView);
}
