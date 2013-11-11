#include "Maze.h"
#include "util.h"

#define DEBUG_LEVEL 0
#include "debug.h"

#include <algorithm>

using std::tuple;
using std::pair;
using std::make_pair;
using std::get;
using std::vector;
using std::get;
using std::string;
using std::stringstream;
using std::endl;
using std::min;
using std::max;

using util::INVALID;

static const double state_size = 0.9;                             // Size of states for rendering.
static const double wall_width = 0.08;                            // Width of walls for rendering.
static const double reward_start_size = 0.15;                     // Size of reward start marker for rendering.
static const double reward_end_size = 0.2;                        // Size of reward end marker for rendering.
static const double reward_end_ratio = 0.5;                       // Length-to-width ratio of reward end marker (arrow) for rendering.
static const double reward_end_notch = 0.25;                      // Depth of the arrow notch.
static const double text_scale = 0.008;                           // Scale factor for text size.
static const QFont  text_font = QFont("Helvetica [Cronyx]", 12);  // Font for texts.
static const double text_center = 0.3;                            // How close the text should be positioned to the midpoint between start and end marker.
static const double border_margin = 0.4*state_size;               // Margin for drawing the border around the maze.
static const double action_line_length_factor = 0.8;              // How long the action line is relative to the state size.
static const double action_point_size_factor = 0.5;               // How large the action point is relative to the state size.
static const bool draw_text = false;                              // Whether to draw texts.

const vector<Maze::wall_t> Maze::walls = {

    /* 2x2 Maze *
    { 1, 3}
    /**/

    /* 3x3 Maze *
    { 0, 1},
    { 0, 3},
    { 2, 1},
    { 2, 5},
    { 6, 3},
    { 6, 7},
    { 8, 7},
    { 8, 5}
    /**/

    /* 3x2 Maze (Paper) *
    { 0, 3},
    { 1, 4},
    { 2, 5}
    /**/

    /* 4x4 Maze (Paper I) *
    { 4, 8},
    { 5, 9},
    { 6,10},
    { 7,11},
    { 1, 2},
    { 5, 6},
    { 9,10},
    {13,14},
    { 8, 9},
    { 2, 3},
    { 6, 7},
    {14,15},
    {11,15},
    { 3, 7}
    /**/

    /* 4x4 Maze (Paper II) *
    {0,4},
    {4,5},
    {8,12},
    {14,15},
    {5,6},
    {2,3}
    /**/

    /* 4x4 Maze (Paper III) *
    { 5, 9},
    {12,13},
    {11,15},
    { 5, 1},
    { 5, 4},
    { 2, 6},
    { 3, 7},
    { 9,10},
    {10,14}
    /**/

    /* 6x6 Maze (Paper) *
    {12, 18},
    {13, 19},
    {14, 20},
    {15, 21},
    {16, 22},
    {17, 23},
    { 2,  3},
    { 8,  9},
    {14, 15},
    {20, 21},
    {26, 27},
    {32, 33},
    {19, 25},
    {25, 26},
    {24, 30},
    {25, 31},
    {27, 33},
    {28, 34},
    {29, 35},
    {27, 28},
    {33, 34},
    {28, 29},
    {34, 35},
    {22, 28},
    {23, 29},
    { 3,  9},
    { 4, 10},
    { 5, 11}
    /**/

    /* 10x10 Maze *
    {  2,  3},
    { 12, 13},
    { 22, 23},
    { 32, 33},
    { 30, 40},
    { 31, 41},
    { 33, 34},
    { 43, 44},
    { 53, 54},
    { 63, 64},
    { 73, 74},
    { 83, 84},
    { 61, 71},
    { 62, 72},
    { 63, 73},
    { 44, 45},
    { 54, 55},
    { 74, 75},
    { 84, 85},
    { 75, 85},
    { 76, 86},
    { 77, 87},
    { 23, 24},
    { 25, 35},
    { 26, 36},
    { 27, 37},
    { 28, 38},
    {  6,  7},
    { 16, 17},
    { 78, 88},
    { 59, 69},
    { 58, 68},
    { 57, 67},
    { 56, 66}
    /**/
};

const vector<Maze::maze_reward_t> Maze::rewards = {
    /* 1x5 Maze *
    { 0, 4, 4, 1, EACH_TIME_NO_PUNISH,   0,   0,   0}
    /**/

    /* 2x2 Maze *
    { 0, 3, 4, 5, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 3, 0, 4, 5, ON_RELEASE_NO_PUNISH, 200, 200,   0},
    { 0, 1, 1, 1, ON_RELEASE_NO_PUNISH,   0, 200,   0},
    { 3, 2, 1, 1, ON_RELEASE_NO_PUNISH,   0,   0, 200}
    /**/

    /* 2x2 Maze */
    { 0, 3, 2, 1, EACH_TIME_NO_PUNISH, 200,   0,   0}
    /**/

    /* 3x3 Maze *
    { 3, 5, 4, 8, ON_RELEASE_NO_PUNISH,   0, 200,   0},
    { 5, 3, 6, 8, ON_RELEASE_NO_PUNISH,   0, 200, 200},
    { 4, 1, 1, 1, ON_RELEASE_NO_PUNISH, 200, 200,   0},
    { 4, 7, 3, 3, ON_RELEASE_NO_PUNISH, 200,   0,   0}
    /**/

    /* 3x2 Maze (Paper) *
    { 3, 0, 5, 1, EACH_TIME_NO_PUNISH, 0, 0, 0},
    { 2, 5, 5, 1, EACH_TIME_NO_PUNISH, 0, 0, 0}
    /**/

    /* 4x4 Maze (Paper I) *
    { 0,  4, 3, 1, ON_RELEASE_NO_PUNISH, 255, 120,  0},
    { 5,  0, 2, 1, ON_RELEASE_NO_PUNISH, 200,   0,  0},
    { 1,  5, 3, 1, ON_RELEASE_NO_PUNISH, 255, 120,  0},
    { 2,  1, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0},
    { 7,  6, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0},
    {11,  7, 4, 1,  EACH_TIME_NO_PUNISH, 255, 200,  0},
    {10, 15, 3, 1,  EACH_TIME_NO_PUNISH, 255, 120,  0},
    {13, 14, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0},
    { 8,  9, 4, 1,  EACH_TIME_NO_PUNISH, 255, 200,  0},
    { 4,  8, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,  0}
    /**/

    /* 4x4 Maze (Paper II) *
    { 0, 8, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    { 1, 4, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    { 8,13, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    {12,14, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    {14,11, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    {15,10, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    {11, 9, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    { 9, 6, 3, 1, ON_RELEASE_NO_PUNISH, 255,   0,   0},
    { 5, 7, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    { 7, 3, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    { 7, 2, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0},
    { 2, 1, 3, 1,  EACH_TIME_NO_PUNISH, 255,   0,   0}
    /**/

    /* 4x4 Maze (Paper III) *
    { 0, 8, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    { 8,13, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    { 9,12, 3, 1,  EACH_TIME_NO_PUNISH, 255, 100,   0},
    {12,14, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {13,15, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {11, 6, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {10, 5, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    { 5, 2, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    { 1, 3, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    { 2, 0, 2, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0}
    /**/

    /* 6x6 Maze (Paper) *
    {25, 31, 2, 1,  EACH_TIME_NO_PUNISH, 100,   0,   0},
    {19, 25, 3, 1,  EACH_TIME_NO_PUNISH,  50,   0,   0},
    {13, 19, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {26, 27, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {28, 29, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {29, 35, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {35, 34, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {33, 21, 2, 1,  EACH_TIME_NO_PUNISH, 100,   0,   0},
    {34, 33, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {21, 23, 2, 1,  EACH_TIME_NO_PUNISH, 100,   0,   0},
    {22, 16, 3, 1,  EACH_TIME_NO_PUNISH,  50,   0,   0},
    {17, 11, 3, 1, ON_RELEASE_NO_PUNISH,  50,   0,   0},
    { 4, 10, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    {11,  4, 3, 1,  EACH_TIME_NO_PUNISH,  50,   0,   0},
    { 9,  8, 1, 1,  EACH_TIME_NO_PUNISH, 200,   0,   0},
    { 7,  6, 3, 1, ON_RELEASE_NO_PUNISH,  50,   0,   0},
    { 2,  7, 2, 1,  EACH_TIME_NO_PUNISH, 100,   0,   0},
    { 0, 12, 2, 1,  EACH_TIME_NO_PUNISH, 100,   0,   0},
    { 1,  0, 3, 1, ON_RELEASE_NO_PUNISH,  50,   0,   0}
    /**/

    /* 10x10 Maze *
    {  0, 21,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 30, 43,  4, 1, ON_RELEASE_NO_PUNISH,   0, 200,   0},
    { 43, 62,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 61, 92, -4, 1, ON_RELEASE_NO_PUNISH,   0, 200,   0},
    { 94, 86,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 87, 66,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 67, 59,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 48, 56,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 55, 36,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 59, 38,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 28,  9,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 18, 37,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 25, 33,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    { 33,  3,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    {  4,  1,  3, 1, ON_RELEASE_NO_PUNISH, 200,   0,   0},
    {  8, 17,  2, 1, ON_RELEASE_NO_PUNISH,   0,   0, 200},
    { 40, 50,  2, 1, ON_RELEASE_NO_PUNISH,   0, 200, 200}, // test
    { 41, 51,  2, 1,  EACH_TIME_NO_PUNISH,   0, 200, 200}, // test
    { 60, 70,  2, 1, ON_RELEASE_PUNISH_FAILURE,   0, 200, 200}, // test
    { 80, 90,  2, 1,  EACH_TIME_PUNISH_FAILURE,   0, 200, 200}  // test
    /**/
};

const vector<Maze::door_t> Maze::doors = {
    /* 2x2 Maze *
    door_t(MazeState(1,0), MazeState(1,1), MazeState(1,1),  RIGHT_BUTTON,-3, color_t(0.0,0.8,0.0) ),
    /**/

    /* 3x2 Maze (Paper) *
    door_t(MazeState(1), MazeState(4), MazeState(2),   UP_BUTTON, -4, color_t(0.0,0.5,0.0) ),
    door_t(MazeState(1), MazeState(4), MazeState(3), DOWN_BUTTON, -4, color_t(0.5,0.0,0.0) )
    /**/

    /* 4x4 Maze (Paper I) *
    door_t(MazeState( 4), MazeState( 8), MazeState( 4),  DOWN_BUTTON,  0, color_t(0.6,0.0,1.0) ),
    door_t(MazeState( 8), MazeState( 9), MazeState( 8),  LEFT_BUTTON, -3, color_t(0.0,0.5,0.0) ),
    door_t(MazeState(13), MazeState(14), MazeState(13), RIGHT_BUTTON,  0, color_t(0.6,0.0,1.0) ),
    door_t(MazeState(14), MazeState(15), MazeState(10),    UP_BUTTON, -3, color_t(0.0,0.5,0.0) ),
    door_t(MazeState(15), MazeState(11), MazeState(15), RIGHT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
    door_t(MazeState(11), MazeState( 7), MazeState(11),    UP_BUTTON,  0, color_t(0.6,0.0,1.0) ),
    door_t(MazeState( 7), MazeState( 3), MazeState( 7), RIGHT_BUTTON, -3, color_t(0.0,0.5,0.0) ),
    door_t(MazeState( 3), MazeState( 7), MazeState( 3), RIGHT_BUTTON, -3, color_t(0.0,0.5,0.0) ),
    door_t(MazeState( 7), MazeState( 6), MazeState( 3),  LEFT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
    door_t(MazeState( 2), MazeState( 1), MazeState( 2),  LEFT_BUTTON,  0, color_t(0.6,0.0,1.0) )

    /**/

    /* 4x4 Maze (Paper II) *
    door_t(MazeState( 0),MazeState( 4),MazeState( 1),    UP_BUTTON, -3, color_t(0.0,0.0,1.0) ),
    door_t(MazeState( 8),MazeState(12),MazeState( 4), RIGHT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
    door_t(MazeState(14),MazeState(15),MazeState(14),  DOWN_BUTTON, -3, color_t(0.0,0.0,1.0) ),
    door_t(MazeState( 5),MazeState( 6),MazeState( 5),  LEFT_BUTTON, -3, color_t(0.0,0.0,1.0) ),
    door_t(MazeState( 3),MazeState( 2),MazeState( 3), RIGHT_BUTTON, -3, color_t(0.0,0.0,1.0) )
    /**/

    /* 4x4 Maze (Paper III) *
    door_t(MazeState(12),MazeState(13),MazeState(13),  DOWN_BUTTON, -2, color_t(0.0,1.0,0.0) ),
    door_t(MazeState(15),MazeState(11),MazeState(15), RIGHT_BUTTON, -2, color_t(0.0,1.0,0.0) ),
    door_t(MazeState( 5),MazeState( 1),MazeState( 5),  DOWN_BUTTON, -2, color_t(0.0,1.0,0.0) )
    /**/

    /* 6x6 Maze (Paper) *
    door_t(MazeState(13), MazeState(19), MazeState(13),  DOWN_BUTTON,-0, color_t(0.0,0.8,0.0) ),
    door_t(MazeState(19), MazeState(25), MazeState(19),    UP_BUTTON,-1, color_t(0.0,0.4,0.0) ),
    door_t(MazeState(25), MazeState(31), MazeState(25),  DOWN_BUTTON,-0, color_t(0.0,0.8,0.0) ),
    door_t(MazeState(26), MazeState(27), MazeState(26), RIGHT_BUTTON,-0, color_t(0.0,0.8,0.0) ),
    door_t(MazeState(27), MazeState(28), MazeState(27), RIGHT_BUTTON,-0, color_t(0.0,0.8,0.0) ),
    door_t(MazeState(28), MazeState(29), MazeState(28), RIGHT_BUTTON,-0, color_t(0.0,0.8,0.0) ),
    door_t(MazeState(29), MazeState(35), MazeState(29),    UP_BUTTON,-1, color_t(0.0,0.4,0.0) ),
    door_t(MazeState(35), MazeState(34), MazeState(35),  DOWN_BUTTON,-1, color_t(0.0,0.4,0.0) ),
    door_t(MazeState(34), MazeState(33), MazeState(34),    UP_BUTTON,-1, color_t(0.0,0.4,0.0) ),
    door_t(MazeState(33), MazeState(27), MazeState(33),  LEFT_BUTTON,-1, color_t(0.0,0.4,0.0) ),
    door_t(MazeState(22), MazeState(16), MazeState(22),    UP_BUTTON,-0, color_t(0.0,0.8,0.0) ),
    door_t(MazeState(10), MazeState( 4), MazeState(11), RIGHT_BUTTON,-4, color_t(0.0,0.2,0.8) ),
    door_t(MazeState( 5), MazeState(11), MazeState( 3),  LEFT_BUTTON,-4, color_t(0.6,0.0,0.8) ),
    door_t(MazeState( 9), MazeState( 8), MazeState( 9),  LEFT_BUTTON,-0, color_t(0.0,0.8,0.0) )
    /**/

    /* 10x10 Maze *
    door_t(MazeState(0,3), MazeState(0,4), MazeState(0,4),  PASS_BUTTON, 0, color_t(1.0,0.5,0.0) ),
    door_t(MazeState(1,3), MazeState(1,4), MazeState(1,5),  STAY_BUTTON, 2, color_t(0.0,1.0,0.0) ),
    door_t(MazeState(2,2), MazeState(3,2), MazeState(2,3),    UP_BUTTON, 2, color_t(0.0,1.0,0.0) ),
    door_t(MazeState(3,3), MazeState(4,3), MazeState(3,3),  DOWN_BUTTON, 3, color_t(0.0,0.5,0.5) ),
    door_t(MazeState(4,5), MazeState(5,5), MazeState(4,6),  LEFT_BUTTON,-5, color_t(0.0,0.0,1.0) ),
    door_t(MazeState(6,5), MazeState(6,6), MazeState(4,7), RIGHT_BUTTON, 5, color_t(1.0,0.0,1.0) )
    /**/
};

Maze::Maze(const double& eps):
    current_instance(nullptr),
    view(nullptr),
    epsilon(eps),
    agent(nullptr)
{
    // setting current state
    current_instance = instance_t::create(action_t::STAY, current_state.state_idx(), reward_t(0));
    current_state = MazeState(Config::maze_x_size/2, Config::maze_y_size/2);
    DEBUG_OUT(1,"Current Maze State: " << current_state << " (Index: " << current_state.state_idx() << ")" );
    set_current_state(current_state.state_idx());

    // TODO: Test if all rewards are actually valid!
}


Maze::~Maze() {
    delete agent;
    delete current_instance;
//    delete button;
//    delete smiley;
}


void Maze::render_initialize(QGraphicsView * v) {

    // Set view
    view = v;

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();
    if(scene==NULL) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

    // Put Border Around Maze
    frame_maze();

    // Render States
    state_rects.clear();
    for(auto observation : observationIt_t::all) {
        render_state(observation);
    }

    // Render Walls
    for( auto w : walls ) {
        render_wall(w);
    }

    // Render Doors
    for( auto d : doors ) {
        render_door(d);
    }

    // Render action line and circle
    QPen action_line_pen(QColor(0,0,0,50),0.1,Qt::SolidLine,Qt::RoundCap);
    QPen action_point_pen(QColor(0,0,0),0.01,Qt::SolidLine,Qt::RoundCap);
    QBrush action_point_brush(QColor(0,0,0,30));
    action_line = scene->addLine(current_state.x(),current_state.y(),current_state.x(),current_state.y(),action_line_pen);
    double ap_size = state_size*action_point_size_factor;
    action_point = scene->addEllipse(current_state.x()-ap_size/2,current_state.y()-ap_size/2,ap_size,ap_size,action_point_pen,action_point_brush);

    // Rewards
    for(maze_reward_t reward : rewards ) {
        render_reward(reward);
    }

    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("agent.svg");
        agent->setScale(0.2);
        QSizeF s = agent->boundingRect().size();
        agent->setPos(current_state.x()-s.width()/2, current_state.y()-s.height()/2);
        agent->setElementId("normal");
    }

    scene->addItem(agent);

    rescale_scene(view);
}


void Maze::render_update(const color_vector_t * color) {
    // set agent position and mirror to make 'stay' actions visible
    QSizeF s = agent->boundingRect().size();
    agent->setPos(current_state.x()-agent->scale()*s.width()/2, current_state.y()-agent->scale()*s.height()/2);
    agent->setElementId(agent->elementId()=="normal" ? "mirrored" : "normal");

    // set action line and circle
    MazeState last_state((current_instance->const_it()-1)->observation);
    double al_length = state_size*action_line_length_factor;
    double ap_size = state_size*action_point_size_factor;
    action_point->setRect(last_state.x()-ap_size/2,last_state.y()-ap_size/2,ap_size,ap_size);
    switch(current_instance->action) {
    case action_t::STAY:
        action_line->setLine(last_state.x(),last_state.y(),last_state.x(),last_state.y());
        break;
    case action_t::UP:
        action_line->setLine(last_state.x(),last_state.y(),last_state.x(),last_state.y()-al_length);
        break;
    case action_t::DOWN:
        action_line->setLine(last_state.x(),last_state.y(),last_state.x(),last_state.y()+al_length);
        break;
    case action_t::LEFT:
        action_line->setLine(last_state.x(),last_state.y(),last_state.x()-al_length,last_state.y());
        break;
    case action_t::RIGHT:
        action_line->setLine(last_state.x(),last_state.y(),last_state.x()+al_length,last_state.y());
        break;
    }

    // show reward by border color
    int border_idx = 0;
    bool matching_reward = false; // safety check
    for(rewardIt_t rewIt=rewardIt_t::first(); rewIt!=INVALID; ++rewIt, ++border_idx) {
        if(current_instance->reward==rewIt) {
            borders[border_idx]->setVisible(true);
            matching_reward = true;
        } else {
            borders[border_idx]->setVisible(false);
        }
    }

    if(!matching_reward) {
        DEBUG_OUT(0,"Error: Current reward is invalid (" << current_instance->reward << ")");
    }

    // Change State Color
    if(color!=nullptr) {
        idx_t col_idx = 0;
        for( auto rect_ptr : state_rects ) {
            rect_ptr->setBrush( QColor( get<COLOR_R>((*color)[col_idx])*255, get<COLOR_G>((*color)[col_idx])*255, get<COLOR_B>((*color)[col_idx])*255 ) );
            ++col_idx;
        }
    }

    rescale_scene(view);
}


void Maze::perform_transition(const action_t& action, std::vector<std::pair<int,int> > * reward_vector) {

    MazeState old_state = current_state; // remember current (old) state

    if(DEBUG_LEVEL>=1) {
        DEBUG_OUT(1,"Current instance: ");
        const_instanceIt_t insIt = current_instance->it();
        for(idx_t k_idx=0; k_idx<(idx_t)Config::k; ++k_idx) {
            DEBUG_OUT(1,"    " << (*insIt) );
            --insIt;
        }
    }

    // perform transition
    probability_t prob_threshold = drand48();
    DEBUG_OUT(2,"Prob threshold = " << prob_threshold);
    probability_t prob_accum = 0;
    bool was_set = false;
    for(observationIt_t observation_to=observationIt_t::first(); observation_to!=INVALID && !was_set; ++observation_to) {
        for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID && !was_set; ++reward) {
            probability_t prob = get_prediction(current_instance, action, observation_to, reward, reward_vector);
            DEBUG_OUT(2,"observation(" << observation_to << "), reward(" << reward << ") --> prob=" << prob);
            prob_accum += prob;
            if(prob_accum>prob_threshold) {
                current_state = MazeState(observation_to);
                current_instance = current_instance->append_instance(action, observation_to, reward);
                was_set = true;
                DEBUG_OUT(2,"CHOOSE");
            }
        }
    }
    if(!was_set) {
        DEBUG_OUT(0, "Error: Unnormalized probabilities [sum(p)=" << prob_accum << "]--> no transition performed." );
    }

    DEBUG_OUT(1, "(" <<
              old_state.x() << "," <<
              old_state.y() << ") + " <<
              action << " ==> (" <<
              current_state.x() << "," <<
              current_state.y() << ")"
        );
}

void Maze::perform_transition(const action_t& action) {
    perform_transition(action, nullptr);
}

void Maze::perform_transition(const action_t& a, observation_t& final_observation, reward_t& r) {
    perform_transition(a);
    final_observation = current_state.state_idx();
    r = current_instance->reward;
}

Maze::probability_t Maze::get_prediction(const instance_t* instance_from, const action_t& action, const observation_t& observation_to, const reward_t& reward) const {
    return get_prediction(instance_from, action, observation_to, reward, nullptr);
}

Maze::probability_t Maze::get_prediction(const instance_t* instance_from, const action_t& action, const observation_t& observation_to, const reward_t& reward, vector<pair<int,int> > * reward_vector) const {

    // state 'from' and 'to'
    MazeState maze_state_from(instance_from->observation);
    MazeState maze_state_to(observation_to);
    int x_from = maze_state_from.x();
    int y_from = maze_state_from.y();

    // states that can in principle be reached (ignoring walls and doors)
    MazeState maze_state_stay(                               x_from   ,                                y_from   );
    MazeState maze_state_left( clamp(0,Config::maze_x_size-1,x_from-1),                                y_from   );
    MazeState maze_state_right(clamp(0,Config::maze_x_size-1,x_from+1),                                y_from   );
    MazeState maze_state_up(                                 x_from   , clamp(0,Config::maze_y_size-1, y_from-1));
    MazeState maze_state_down(                               x_from   , clamp(0,Config::maze_y_size-1, y_from+1));

    // check if state_to can be reached at all
    if( maze_state_to!=maze_state_stay  &&
        maze_state_to!=maze_state_left  &&
        maze_state_to!=maze_state_right &&
        maze_state_to!=maze_state_up    &&
        maze_state_to!=maze_state_down ) {
        return 0;
    }

    // check and remember blocking/unblocking (store references as array for
    // later manipulation, ignore 'stay' which cannot be blocked/unblocked)
    MazeState reachable_states[4] = { maze_state_left, maze_state_right, maze_state_up, maze_state_down };
    bool  left_unblocked_by_door = false;
    bool right_unblocked_by_door = false;
    bool    up_unblocked_by_door = false;
    bool  down_unblocked_by_door = false;
    bool* unblocked_by_door[4] = { &left_unblocked_by_door, &right_unblocked_by_door, &up_unblocked_by_door, &down_unblocked_by_door };
    bool  left_blocked_by_wall = false;
    bool right_blocked_by_wall = false;
    bool    up_blocked_by_wall = false;
    bool  down_blocked_by_wall = false;
    bool* blocked_by_wall[4] = { &left_blocked_by_wall, &right_blocked_by_wall, &up_blocked_by_wall, &down_blocked_by_wall };

    // check for doors (unblocking doors take precedence over closed doors)
    for( auto d : doors ) {
        MazeState s1 = get<DOOR_STATE_FROM>(d);
        MazeState s2 = get<DOOR_STATE_TO>(d);
        MazeState s3 = get<DOOR_KEY_STATE>(d);
        KEY_TYPE kt =  get<DOOR_KEY>(d);
        idx_t delay =  get<DOOR_TIME_DELAY>(d);

        // iterate through reachable state
        for(int idx=0; idx<4; ++idx) {
            MazeState tmp_maze_state_to = reachable_states[idx];

            // check if transition was unblocked already
            if(*unblocked_by_door[idx]) {
                continue;
            }

            // check if the door connects the two states
            if( (tmp_maze_state_to==s1 && maze_state_from==s2) || (tmp_maze_state_to==s2 && maze_state_from==s1) ) {

                // check if key was activated
                idx_t steps_in_past = 0;
                action_t past_action = action; // action must be set one
                                               // iteration earlier because some
                                               // times (e.g. in look-ahead
                                               // search tree) one cannot go
                                               // back to the future because it
                                               // is not uniquely defines
                for( const_instanceIt_t insIt = instance_from->const_it(); insIt!=INVALID && steps_in_past<=abs(delay); past_action=insIt->action, --insIt, ++steps_in_past) {

                    // check if delay matches
                    if(delay<0 || steps_in_past==delay) { // exact match for positve delay less-equal match for negative

                        // check if key state was visited at that time
                        if(MazeState(insIt->observation)==s3) {

                            // check action
                            switch(kt) {
                            case PASS_BUTTON:
                                // action does not matter
                                *unblocked_by_door[idx]=true;
                                break;
                            case STAY_BUTTON:
                                if(past_action==(action_t)action_t::STAY) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case UP_BUTTON:
                                if(past_action==(action_t)action_t::UP) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case DOWN_BUTTON:
                                if(past_action==(action_t)action_t::DOWN) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case LEFT_BUTTON:
                                if(past_action==(action_t)action_t::LEFT) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case RIGHT_BUTTON:
                                if(past_action==(action_t)action_t::RIGHT) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    // check for walls
    for(auto w : walls) {
        MazeState s1(w[0]);
        MazeState s2(w[1]);

        // iterate through reachable states
        for(int idx=0; idx<4; ++idx) {
            MazeState tmp_maze_state_to = reachable_states[idx];

            // ignore wall if transition was unblocked by door or already blocked by wall
            // if(*unblocked_by_door[idx] || *blocked_by_wall[idx]) {
            //     continue;
            // }

            // check if wall is between the two states
            if( (maze_state_from==s1 && tmp_maze_state_to==s2) || (maze_state_from==s2 && tmp_maze_state_to==s1) ) {
                *blocked_by_wall[idx] = true;
            }
        }
    }

    // determine effective states (by considering blocking/unblocking)
    MazeState effective_maze_state_stay  = maze_state_stay;
    MazeState effective_maze_state_left  = ( left_unblocked_by_door || ! left_blocked_by_wall) ? maze_state_left  : maze_state_stay;
    MazeState effective_maze_state_right = (right_unblocked_by_door || !right_blocked_by_wall) ? maze_state_right : maze_state_stay;
    MazeState effective_maze_state_up    = (   up_unblocked_by_door || !   up_blocked_by_wall) ? maze_state_up    : maze_state_stay;
    MazeState effective_maze_state_down  = ( down_unblocked_by_door || ! down_blocked_by_wall) ? maze_state_down  : maze_state_stay;

    // determine state transition probabilites via effective states
    probability_t prob = 0;
    {
        // random action
        if(maze_state_to==effective_maze_state_stay ) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_left ) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_right) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_up   ) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_down ) prob += epsilon/5;

        // intended action
        switch(action) {
        case action_t::STAY:
            if(maze_state_to==effective_maze_state_stay ) prob += 1-epsilon;
            break;
        case action_t::LEFT:
            if(maze_state_to==effective_maze_state_left ) prob += 1-epsilon;
            break;
        case action_t::RIGHT:
            if(maze_state_to==effective_maze_state_right) prob += 1-epsilon;
            break;
        case action_t::UP:
            if(maze_state_to==effective_maze_state_up   ) prob += 1-epsilon;
            break;
        case action_t::DOWN:
            if(maze_state_to==effective_maze_state_down ) prob += 1-epsilon;
            break;
        default:
            DEBUG_OUT(0,"Error: unknown action");
            return 0;
        }

        if(prob==0) {
            return 0;
        }
    }

    // calculate accumulated reward
    reward_t accumulated_reward = 0;
    if(reward_vector!=nullptr) {
        reward_vector->assign(rewards.size(),make_pair(0,0));
    }
    for(int r_idx=0; r_idx<(int)rewards.size(); ++r_idx) {
        maze_reward_t r = rewards[r_idx];
        MazeState activate_state(r[REWARD_ACTIVATION_STATE]);
        MazeState receive_state(r[REWARD_RECEIVE_STATE]);
        idx_t delay = r[REWARD_TIME_DELAY];
        REWARD_ACTIVATION_TYPE rat = (REWARD_ACTIVATION_TYPE)r[REWARD_ACTIVATION];

        // check if reward could be received
        bool receive_reward = false;
        if(maze_state_to==receive_state) {
            receive_reward = true;
        }

        // check reward only if it can be received or activation type is with
        // punishment for failure
        if(receive_reward || rat==EACH_TIME_PUNISH_FAILURE || rat==ON_RELEASE_PUNISH_FAILURE) {
            idx_t steps_in_past = 1;
            bool reward_invalidated = false;
            for( const_instanceIt_t insIt = instance_from->const_it(); insIt!=INVALID && steps_in_past<=abs(delay); --insIt, ++steps_in_past) {

                // check if delay matches
                bool delay_matches = false;
                if(delay<0 || steps_in_past==delay) {
                    // exact match for positve delay, less-equal match for negative
                    delay_matches = true;
                }

                // check if agent was on activation state
                bool activation_state = false;
                if(MazeState(insIt->observation)==activate_state) {
                    activation_state = true;
                }

                // handle different types of rewards
                switch(rat) {
                case EACH_TIME_NO_PUNISH:
                    if(activation_state && receive_reward && delay_matches ) {
                        // receive reward whenever everything matches
                        accumulated_reward += r[REWARD_VALUE];
                        if(reward_vector!=nullptr) {
                            ++((*reward_vector)[r_idx].first);
                        }
                    }
                    break;
                case ON_RELEASE_NO_PUNISH:
                    if(activation_state && receive_reward && delay_matches && !reward_invalidated) {
                        // successfully receive reward
                        accumulated_reward += r[REWARD_VALUE];
                        if(reward_vector!=nullptr) {
                            ++((*reward_vector)[r_idx].first);
                        }
                    }
                    if(activation_state) {
                        // agent passed activation state and invalidated later
                        // rewards (even if it received this one)
                        reward_invalidated = true;
                    }
                    break;
                case EACH_TIME_PUNISH_FAILURE:
                    if(activation_state && delay_matches ) {
                        // receive reward whenever everything matches, punish if
                        // not on receive-state
                        if(receive_reward) {
                            accumulated_reward += r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].first);
                            }
                        } else {
                            accumulated_reward -= r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].second);
                            }
                        }
                    }
                    break;
                case ON_RELEASE_PUNISH_FAILURE:
                    if(activation_state && delay_matches && !reward_invalidated) {
                        // receive reward when everything matches and reward was
                        // not invalidated, punish if not on receive-state
                        if(receive_reward) {
                            accumulated_reward += r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].first);
                            }
                        } else {
                            accumulated_reward -= r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].second);
                            }
                        }
                    }
                    if(activation_state) {
                        // agent passed activation state and invalidated later
                        // rewards (even if it received this one or got punished
                        // for missing it)
                        reward_invalidated = true;
                    }
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }

            }
        }
    }

    // check for matching reward
    if(reward!=accumulated_reward) {
        return 0;
    }

    return prob;
}

void Maze::get_features(std::vector<Feature*> & basis_features, LEARNER_TYPE type) const {
    for(int k_idx = 0; k_idx>=-(int)Config::k; --k_idx) {
        if((type==CRF_LEARNER) ||
           (type==UTREE_VALUE_LEARNER && k_idx>-(int)Config::k) ||
           (type==UTREE_OBSERVATION_REWARD_LEARNER && k_idx>-(int)Config::k) ||
           (type==LINEAR_Q_LEARNER)) {
            // actions
            for(action_t action : actionIt_t::all) {
                ActionFeature * action_feature = ActionFeature::create(action,k_idx);
                basis_features.push_back(action_feature);
            }
        }
        if((type==CRF_LEARNER) ||
           (type==UTREE_VALUE_LEARNER && k_idx>-(int)Config::k) ||
           (type==UTREE_OBSERVATION_REWARD_LEARNER && k_idx>-(int)Config::k) ||
           (type==LINEAR_Q_LEARNER && k_idx<0)) {
            // observations
            for(observation_t observation : observationIt_t::all) {
                ObservationFeature * observation_feature = ObservationFeature::create(observation,k_idx);
                basis_features.push_back(observation_feature);
            }
        }
        if((type==CRF_LEARNER && k_idx==0) ||
           (type==UTREE_VALUE_LEARNER && k_idx>-(int)Config::k) ||
           (type==UTREE_OBSERVATION_REWARD_LEARNER && k_idx>-(int)Config::k) ||
           (type==LINEAR_Q_LEARNER && false)) {
            // reward
            for(reward_t reward : rewardIt_t::all) {
                RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
                basis_features.push_back(reward_feature);
            }
        }
    }

    if(type==CRF_LEARNER) {
        // relative observation features
        RelativeObservationFeature * relative_observation_feature;
        relative_observation_feature = RelativeObservationFeature::create(1,0,-1,0);
        basis_features.push_back(relative_observation_feature);
        relative_observation_feature = RelativeObservationFeature::create(0,1,-1,0);
        basis_features.push_back(relative_observation_feature);
        relative_observation_feature = RelativeObservationFeature::create(-1,0,-1,0);
        basis_features.push_back(relative_observation_feature);
        relative_observation_feature = RelativeObservationFeature::create(0,-1,-1,0);
        basis_features.push_back(relative_observation_feature);
        relative_observation_feature = RelativeObservationFeature::create(0,0,-1,0);
        basis_features.push_back(relative_observation_feature);
    }
    if(type==LINEAR_Q_LEARNER) {
        // also add a unit feature
        ConstFeature * const_feature = ConstFeature::create(1);
        basis_features.push_back(const_feature);
        DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
    }
}

void Maze::print_reward_activation_on_random_walk(const int& walk_length) {
    // collect data
    vector<pair<long,long> > reward_vector(rewards.size(),make_pair(0,0));
    for(int i=0; i<walk_length; ++i) {
        action_t action = action_t::random_action();
        vector<pair<int,int> > tmp_reward_vector;
        perform_transition(action,&tmp_reward_vector);
        for(int r_idx=0; r_idx<(int)reward_vector.size(); ++r_idx) {
            reward_vector[r_idx].first += tmp_reward_vector[r_idx].first;
            reward_vector[r_idx].second += tmp_reward_vector[r_idx].second;
        }
    }

    // print relativ counts
    DEBUG_OUT(0,"Relative frequencies for reward activation on a length " << walk_length << " random walk");
    for(int r_idx=0; r_idx<(int)reward_vector.size(); ++r_idx) {
        DEBUG_OUT(0,"    Reward " << r_idx <<
                  " (" << (observation_t)rewards[r_idx][REWARD_ACTIVATION_STATE] << "," << (observation_t)rewards[r_idx][REWARD_RECEIVE_STATE] <<
                  ")	p+ = " << (double)reward_vector[r_idx].first/walk_length <<
                  "	p- = " << (double)reward_vector[r_idx].second/walk_length
            );
    }
}

void Maze::set_epsilon(const double& e) {
    epsilon = e;
}

void Maze::set_current_state(const observation_t& observation) {
    current_state = MazeState(observation);
    for(idx_t k_idx=0; k_idx<(idx_t)Config::k; ++k_idx) {
        current_instance = current_instance->append_instance(action_t::STAY, current_state.state_idx(), reward_t(0));
    }
    DEBUG_OUT(1,"Set current state to (" << current_state.x() << "," << current_state.y() << ")");
}

string Maze::get_rewards() {
    stringstream ss;
    for(int r_idx=0; r_idx<(int)rewards.size(); ++r_idx) {
        ss << "Reward " << r_idx << endl;
        ss << "    ACTIVATION_STATE : " << (observation_t)rewards[r_idx][REWARD_ACTIVATION_STATE] << endl;
        ss << "    RECEIVE_STATE    : " << (observation_t)rewards[r_idx][REWARD_RECEIVE_STATE] << endl;
        ss << "    TIME_DELAY       : " << (int)rewards[r_idx][REWARD_TIME_DELAY] << endl;
        ss << "    reward           : " << (reward_t)rewards[r_idx][REWARD_VALUE] << endl;
        ss << "    activation       : " << reward_activation_type_str((REWARD_ACTIVATION_TYPE)rewards[r_idx][REWARD_ACTIVATION]) << endl;
    }
    return ss.str();
}

string Maze::get_walls() {
    stringstream ss;
    for(int w_idx=0; w_idx<(int)walls.size(); ++w_idx) {
        ss << "Wall " << w_idx << ": (" << walls[w_idx][0] << "|" << walls[w_idx][1] << ")" << endl;
    }
    return ss.str();
}

string Maze::get_doors() {
    stringstream ss;
    for(door_t d : doors) {
        ss << "Door (" << get<DOOR_STATE_FROM>(d) << "/" << get<DOOR_STATE_TO>(d) << ")" << endl;
        ss << "    key : " << get<DOOR_KEY_STATE>(d) << " : ";
        switch(get<DOOR_KEY>(d)) {
        case PASS_BUTTON:
            ss << "PASS_BUTTON";
            break;
        case STAY_BUTTON:
            ss << "STAY_BUTTON";
            break;
        case UP_BUTTON:
            ss << "UP_BUTTON";
            break;
        case DOWN_BUTTON:
            ss << "DOWN_BUTTON";
            break;
        case LEFT_BUTTON:
            ss << "LEFT_BUTTON";
            break;
        case RIGHT_BUTTON:
            ss << "RIGHT_BUTTON";
            break;
        default:
            DEBUG_DEAD_LINE;
        }
        ss << endl;
        ss << "    dt : " << get<DOOR_TIME_DELAY>(d) << endl;
    }
    return ss.str();
}

void Maze::frame_maze() {

    QGraphicsScene * scene = view->scene();

    MazeState first_maze_state(observationIt_t::first());
    MazeState last_maze_state(observationIt_t::last());
    double border_x = first_maze_state.x()-state_size/2 - border_margin;
    double border_y = first_maze_state.y()-state_size/2 - border_margin;
    double border_width = last_maze_state.x()+state_size/2 - border_x + border_margin;
    double border_height = last_maze_state.y()+state_size/2 - border_y + border_margin;
    double reward_magnitude = max(fabs(reward_t::max_reward), fabs(reward_t::min_reward));
    for(rewardIt_t rewIt=rewardIt_t::first(); rewIt!=INVALID; ++rewIt) {
        double intensity = ((reward_t)rewIt)/reward_magnitude;
        bool positive = intensity>=0;
        intensity = fabs(intensity);
        if(intensity!=0) {
            // to clearly distinguish reward from non-reward
            intensity = intensity/2 + 0.5;
        }
        intensity *= 255;
        QPen border_pen(QColor(positive?0:intensity,positive?intensity:0,0), 0.04, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        borders.push_back( scene->addRect( border_x, border_y, border_width, border_height, border_pen, QBrush(QColor(255,255,255)) ) );
        if(intensity!=0) {
            borders.back()->setVisible(false);
        }
    }
}

void Maze::render_state(observation_t s) {

    QGraphicsScene * scene = view->scene();

    QPen state_pen(QColor(0,0,0), 0.02, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    MazeState maze_state(s);
    QGraphicsRectItem * rect = scene->addRect( maze_state.x()-state_size/2,
                                               maze_state.y()-state_size/2,
                                               state_size,
                                               state_size,
                                               state_pen,
                                               QBrush(QColor(230,230,230))
        );
    state_rects.push_back(rect);
}

void Maze::render_wall(wall_t w) {

    QGraphicsScene * scene = view->scene();

    QPen wall_pen(QColor(50,50,50), 0.02, Qt::SolidLine, Qt::RoundCap);
    QBrush wall_brush(QColor(50,50,50));

    MazeState maze_state_1(w[0]);
    MazeState maze_state_2(w[1]);
    idx_t x_1 = maze_state_1.x();
    idx_t y_1 = maze_state_1.y();
    idx_t x_2 = maze_state_2.x();
    idx_t y_2 = maze_state_2.y();
    if( abs(x_1-x_2)==1 && abs(y_1-y_2)==0 ) {
        idx_t x_min = min<idx_t>(x_1,x_2);
        idx_t y = y_1;
        scene->addRect( x_min+0.5-wall_width/2, y-0.5, wall_width, 1, wall_pen, wall_brush );
        scene->addEllipse( x_min+0.5-wall_width/2, y-0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
        scene->addEllipse( x_min+0.5-wall_width/2, y+0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
    } else if( abs(x_1-x_2)==0 && abs(y_1-y_2)==1 ) {
        idx_t x = x_1;
        idx_t y_min = min<idx_t>(y_1,y_2);
        scene->addRect( x-0.5, y_min+0.5-wall_width/2, 1, wall_width, wall_pen, wall_brush );
        scene->addEllipse( x-0.5-wall_width/2, y_min+0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
        scene->addEllipse( x+0.5-wall_width/2, y_min+0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
    } else {
        DEBUG_OUT(0,"Error: No wall possible between (" <<
                  x_1 << "," << y_1 << ") and (" <<
                  x_2 << "," << y_2 << ")" );
    }
}

void Maze::render_door(door_t dt) {

    QGraphicsScene * scene = view->scene();

    MazeState s1 = get<DOOR_STATE_FROM>(dt);
    MazeState s2 = get<DOOR_STATE_TO>(dt);
    MazeState s3 = get<DOOR_KEY_STATE>(dt);
    KEY_TYPE kt =  get<DOOR_KEY>(dt);
    idx_t delay =  get<DOOR_TIME_DELAY>(dt);
    color_t c = get<DOOR_COLOR>(dt);

    QColor door_color(255*get<COLOR_R>(c),255*get<COLOR_G>(c),255*get<COLOR_B>(c));
    QPen door_pen(door_color, 0.05, Qt::SolidLine, Qt::RoundCap);
    QBrush door_brush(door_color);

    // draw door
    idx_t x1 = s1.x();
    idx_t y1 = s1.y();
    idx_t x2 = s2.x();
    idx_t y2 = s2.y();
    if( abs(x1-x2)==1 && abs(y1-y2)==0 ) {
        idx_t x_min = min<idx_t>(x1,x2);
        idx_t y = y1;
        scene->addLine( x_min+0.5, y-0.3, x_min+0.7, y+0.3, door_pen );
    } else if( abs(x1-x2)==0 && abs(y1-y2)==1 ) {
        idx_t x = x1;
        idx_t y_min = min<idx_t>(y1,y2);
        scene->addLine( x+0.3, y_min+0.5, x-0.3, y_min+0.7, door_pen );
    } else {
        DEBUG_OUT(0,"Error: No door possible between (" <<
                  x1 << "," << y1 << ") and (" <<
                  x2 << "," << y2 << ")" );
    }

    // draw key
    idx_t x3 = s3.x();
    idx_t y3 = s3.y();
    double text_x, text_y;
    switch(kt) {
    case PASS_BUTTON:
        scene->addEllipse( x3-0.1, y3-0.1, 0.2, 0.2, door_pen );
        text_x = x3;
        text_y = y3;
        break;
    case STAY_BUTTON:
        scene->addEllipse( x3-0.1, y3-0.1, 0.2, 0.2, door_pen, door_brush );
        text_x = x3;
        text_y = y3;
        break;
    case UP_BUTTON:
        scene->addEllipse( x3-0.1, y3-state_size/2, 0.2, 0.05, door_pen, door_brush );
        text_x = x3;
        text_y = y3-state_size/4;
        break;
    case DOWN_BUTTON:
        scene->addEllipse( x3-0.1, y3+state_size/2-0.05, 0.2, 0.05, door_pen, door_brush );
        text_x = x3;
        text_y = y3+state_size/4;
        break;
    case LEFT_BUTTON:
        scene->addEllipse( x3-state_size/2, y3-0.1, 0.05, 0.2, door_pen, door_brush );
        text_x = x3-state_size/4;
        text_y = y3;
        break;
    case RIGHT_BUTTON:
        scene->addEllipse( x3+state_size/2-0.05, y3-0.1, 0.05, 0.2, door_pen, door_brush );
        text_x = x3+state_size/4;
        text_y = y3;
        break;
    }

    // add text
    if(draw_text) {
        QGraphicsTextItem * txt = scene->addText(
            QString("t=%1").arg(QString::number(delay)),
            text_font
            );
        QRectF box = txt->boundingRect();
        txt->setPos(
            text_x-text_scale*box.width()/2,
            text_y-text_scale*box.height()/2
            );
        txt->setScale(text_scale);
        txt->setDefaultTextColor(door_color);
    }
}

void Maze::render_reward(maze_reward_t r) {

    QGraphicsScene * scene = view->scene();

    MazeState maze_state_1((int)r[REWARD_ACTIVATION_STATE]);
    MazeState maze_state_2((int)r[REWARD_RECEIVE_STATE]);
    double x_start   = maze_state_1.x();
    double y_start   = maze_state_1.y();
    double x_end     = maze_state_2.x();
    double y_end     = maze_state_2.y();
    double x_shift   = -(y_end-y_start)/5;
    double y_shift   = (x_end-x_start)/5;
    double x_control = (x_start+x_end)/2+x_shift;
    double y_control = (y_start+y_end)/2+y_shift;
    QColor color( (int)r[REWARD_R], (int)r[REWARD_G], (int)r[REWARD_B]);
    QPen   arc_pen( color,0.02,Qt::SolidLine,Qt::RoundCap );
    QPen   marker_pen( color,0.02,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin );
    QBrush brush( color );
    QBrush no_brush( Qt::NoBrush );

    // arc
    QPainterPath arc_path;
    arc_path.moveTo(x_start, y_start);
    arc_path.quadTo( x_control, y_control, x_end, y_end );
    scene->addPath(arc_path,arc_pen,QBrush(QColor(0,0,0,0)));

    // start
    switch((REWARD_ACTIVATION_TYPE)r[REWARD_ACTIVATION]) {
    case EACH_TIME_NO_PUNISH:
        scene->addEllipse(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            brush
            );
        break;
    case EACH_TIME_PUNISH_FAILURE:
        scene->addRect(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            brush
            );
        break;
    case ON_RELEASE_NO_PUNISH:
        scene->addEllipse(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            no_brush
            );
        break;
    case ON_RELEASE_PUNISH_FAILURE:
        scene->addRect(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            no_brush
            );
        break;
    default:
        DEBUG_DEAD_LINE;
    }

    //========================//
    // drawing the end marker //
    //========================//
    // normal vector in end direction
    double end_vector_x = x_end-x_control;
    double end_vector_y = y_end-y_control;
    double end_vector_norm = sqrt( pow(end_vector_x,2) + pow(end_vector_y,2) );
    end_vector_x /= end_vector_norm;
    end_vector_y /= end_vector_norm;
    // normal vector perpendicular to end direction
    double end_cross_vector_x = +end_vector_y;
    double end_cross_vector_y = -end_vector_x;
    // draw arrow
    QPainterPath end_path;
    end_path.moveTo(x_end, y_end); // point
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x+end_cross_vector_x*reward_end_ratio/2),
        y_end-reward_end_size*(end_vector_y+end_cross_vector_y*reward_end_ratio/2)
        ); // first wing
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x*(1-reward_end_notch)),
        y_end-reward_end_size*(end_vector_y*(1-reward_end_notch))
        ); // notch
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x-end_cross_vector_x*reward_end_ratio/2),
        y_end-reward_end_size*(end_vector_y-end_cross_vector_y*reward_end_ratio/2)
        ); // second wing
    end_path.lineTo(x_end, y_end); // close at point
    scene->addPath(end_path,marker_pen,QBrush(color));

    // text
    if(draw_text) {
        double mid_point_x = x_start + (x_end - x_start)/2;
        double mid_point_y = y_start + (y_end - y_start)/2;
        idx_t time_delay = (idx_t)r[REWARD_TIME_DELAY];
        reward_t reward = (reward_t)r[REWARD_VALUE];
        QGraphicsTextItem * txt = scene->addText(
            QString("t=%1,r=%2").arg(QString::number(time_delay)).arg(QString::number(reward)),
            text_font
            );
        QRectF box = txt->boundingRect();
        txt->setPos(
            x_control+(mid_point_x-x_control)*text_center-text_scale*box.width()/2,
            y_control+(mid_point_y-y_control)*text_center-text_scale*box.height()/2
            );
        txt->setScale(text_scale);
        txt->setDefaultTextColor(color);
    }
}

void Maze::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}

const char* Maze::reward_activation_type_str(REWARD_ACTIVATION_TYPE ra) {
    switch(ra) {
    case EACH_TIME_NO_PUNISH:
        return "EACH_TIME_NO_PUNISH";
    case ON_RELEASE_NO_PUNISH:
        return "ON_RELEASE_NO_PUNISH";
    case EACH_TIME_PUNISH_FAILURE:
        return "EACH_TIME_PUNISH_FAILURE";
    case ON_RELEASE_PUNISH_FAILURE:
        return "ON_RELEASE_PUNISH_FAILURE";
    default:
        DEBUG_DEAD_LINE;
        return "Error!";
    }
}
