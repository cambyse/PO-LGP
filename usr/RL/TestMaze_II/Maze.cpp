
#include "Maze.h"
#include "util.h"

#define DEBUG_LEVEL 0
#include "debug.h"

const double Maze::state_size = 0.9;
const double Maze::wall_width = 0.05;
const double Maze::reward_start_size = 0.1;

using util::min;
using util::max;

const Maze::idx_t Maze::walls[walls_n][2] = {
        /* 2x2 Maze */
        { 0, 1}

        /* 4x4 Maze */
//        { 6, 7},
//        {12,13},
//        { 4, 5},
//        { 6,10},
//        {10,11}
};

const Maze::idx_t Maze::rewards[rewards_n][6] = {
        /* {  activation state,  receive state, time delay, r, g, b} */

        /* 2x2 Maze */
        { 3, 0, 2, 0, 0, 200}

        /* 4x4 Maze */
//        {  4,  2,  3, 200,   0,   0},
//        {  6,  7,  3, 200, 200,   0},
//        { 11, 14,  2,   0, 200,   0},
//        { 13,  8,  2,   0, 200, 200},
//        {  8,  1,  3,   0,   0, 200}
        //            {  8,  4,  4, 200,   0, 200}
};

Maze::Maze(const double& eps):
        time_delay(Data::k),
//        reward_active(false),
        epsilon(eps),
//        button(NULL), smiley(NULL),
        agent(NULL)
{

    if(time_delay<=0) {
        DEBUG_OUT(0,"Error: Time delay must be larger than zero --> setting to one");
        time_delay = 1;
    }

    // setting button and smiley state
//    if(Data::maze_x_dim>0 || Data::maze_y_dim>0) {
//        button_state = MazeState(Data::maze_x_dim-1,Data::maze_y_dim-1);
//    } else {
//        button_state = MazeState(0,0);
//    }
//    smiley_state = MazeState(0,0);

    // setting current state
    current_state = MazeState(Data::maze_x_dim/2, Data::maze_y_dim/2);
    for(idx_t k_idx=0; k_idx<(idx_t)Data::k; ++k_idx) current_k_mdp_state.new_state(Data::STAY, current_state.state_idx(), Data::min_reward);
}


Maze::~Maze() {
    delete agent;
//    delete button;
//    delete smiley;
}


void Maze::render_initialize(QGraphicsView * view) {
    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();
    if(scene==NULL) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

    for(state_t state=0; state<(idx_t)Data::state_n; ++state) {
        MazeState maze_state(state);
        scene->addRect( maze_state.x()-state_size/2, maze_state.y()-state_size/2, state_size, state_size, QPen(QColor(0,0,0)), QBrush(QColor(230,230,230)) );
    }

    // initialize and render button state
//    if(!button) {
//        button = new QGraphicsSvgItem("./button.svg");
//        QSizeF s = button->boundingRect().size();
//        button->setTransformOriginPoint(s.width()/2,s.height()/2);
//        button->setScale(0.2);
//        button->setPos(button_state.x()-s.width()/2,button_state.y()-s.height()/2);
//    }
//    button->setElementId(current_state==button_state ? "active" : "passive");
//    scene->addItem(button);

    // initialize and render smiley state
//    if(!smiley) {
//        smiley = new QGraphicsSvgItem("./smiley.svg");
//        QSizeF s = smiley->boundingRect().size();
//        smiley->setTransformOriginPoint(s.width()/2,s.height()/2);
//        smiley->setScale(0.2);
//        smiley->setPos(smiley_state.x()-s.width()/2, smiley_state.y()-s.height()/2);
//    }
//    smiley->setElementId( reward_active ? "active" : "passive");
//    scene->addItem(smiley);

    // walls
    for(idx_t idx=0; idx<(idx_t)walls_n; ++idx) {
        QColor color(50,50,50);
        MazeState maze_state_1(Data::state_from_idx(walls[idx][0]));
        MazeState maze_state_2(Data::state_from_idx(walls[idx][1]));
        idx_t x_1 = maze_state_1.x();
        idx_t y_1 = maze_state_1.y();
        idx_t x_2 = maze_state_2.x();
        idx_t y_2 = maze_state_2.y();
        if( abs(x_1-x_2)==1 && abs(y_1-y_2)==0 ) {
            idx_t x_min = min<idx_t>(x_1,x_2);
            idx_t y = y_1;
            scene->addRect( x_min+0.5-wall_width/2, y-0.5, wall_width, 1, QPen(color), QBrush(color) );
            scene->addEllipse( x_min+0.5-wall_width/2, y-0.5-wall_width/2, wall_width, wall_width, QPen(color), QBrush(color) );
            scene->addEllipse( x_min+0.5-wall_width/2, y+0.5-wall_width/2, wall_width, wall_width, QPen(color), QBrush(color) );
        } else if( abs(x_1-x_2)==0 && abs(y_1-y_2)==1 ) {
            idx_t x = x_1;
            idx_t y_min = min<idx_t>(y_1,y_2);
            scene->addRect( x-0.5, y_min+0.5-wall_width/2, 1, wall_width, QPen(color), QBrush(color) );
            scene->addEllipse( x-0.5-wall_width/2, y_min+0.5-wall_width/2, wall_width, wall_width, QPen(color), QBrush(color) );
            scene->addEllipse( x+0.5-wall_width/2, y_min+0.5-wall_width/2, wall_width, wall_width, QPen(color), QBrush(color) );
        } else {
            DEBUG_OUT(0,"Error: No wall possible between (" <<
                    x_1 << "," << y_1 << ") and (" <<
                    x_2 << "," << y_2 << ")" );
        }
    }

    // rewards
    for(idx_t idx=0; idx<(idx_t)rewards_n; ++idx) {
        MazeState maze_state_1(Data::state_from_idx(rewards[idx][0]));
        MazeState maze_state_2(Data::state_from_idx(rewards[idx][1]));
        double x_start   = maze_state_1.x();
        double y_start   = maze_state_1.y();
        double x_end     = maze_state_2.x();
        double y_end     = maze_state_2.y();
        double x_shift   = -(y_end-y_start)/5;
        double y_shift   = (x_end-x_start)/5;
        double x_control = (x_start+x_end)/2+x_shift;
        double y_control = (y_start+y_end)/2+y_shift;
        QColor color(rewards[idx][3],rewards[idx][4],rewards[idx][5]);
        QPen   pen(   color,0.02,Qt::SolidLine,Qt::RoundCap );
        QBrush brush( color );
        QPainterPath path;
        path.moveTo(x_start, y_start);
        path.quadTo( x_control, y_control, x_end, y_end );
        scene->addPath(path,pen,QBrush(QColor(0,0,0,0)));
        scene->addEllipse(
                x_start-reward_start_size/2,
                y_start-reward_start_size/2,
                reward_start_size,
                reward_start_size,
                pen,
                brush
        );
        double scale = 0.02;
        QGraphicsTextItem * txt = scene->addText(QString::number(rewards[idx][2]),QFont("",12));
        QRectF box = txt->boundingRect();
        txt->setPos(x_control-scale*box.width()/2,y_control-scale*box.height()/2);
        txt->setScale(scale);
        txt->setDefaultTextColor(color);
    }

    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("./agent.svg");
        QSizeF s = agent->boundingRect().size();
        agent->setTransformOriginPoint(s.width()/2,s.height()/2);
        agent->setScale(0.2);
        agent->setPos(current_state.x()-s.width()/2, current_state.y()-s.height()/2);
    }

    scene->addItem(agent);

    rescale_scene(view);
}


void Maze::render_update(QGraphicsView * view) {
//    button->setElementId(current_state==button_state ? "active" : "passive");
//    smiley->setElementId( reward_active ? "active" : "passive");
    QSizeF s = agent->boundingRect().size();
    agent->setPos(current_state.x()-s.width()/2, current_state.y()-s.height()/2);
    rescale_scene(view);
}


void Maze::perform_transition(const action_t& action) {
    MazeState old_state = current_state; // remember current (old) state

    // perform transition
    probability_t prob_threshold = drand48();
    DEBUG_OUT(2,"Prob threshold = " << prob_threshold);
    probability_t prob_accum = 0;
    bool was_set = false;
    for(Data::state_idx_t state_idx_to=0; state_idx_to<(idx_t)Data::state_n && !was_set; ++state_idx_to) {
        state_t state_to = Data::state_from_idx(state_idx_to);
        for(Data::reward_idx_t reward_idx=0; reward_idx<(idx_t)Data::reward_n && !was_set; ++reward_idx) {
            reward_t reward = Data::reward_from_idx(reward_idx);
            probability_t prob = get_prediction(current_k_mdp_state.get_k_mdp_state(), action, state_to, reward);
            DEBUG_OUT(2,"state(" << state_to << "), reward(" << reward << ") --> prob=" << prob);
            prob_accum += prob;
            if(prob_accum>prob_threshold) {
//                reward_active = current_k_mdp_state.get_k_mdp_state()[time_delay-1].state==button_state.state_idx();
                current_state = state_to;
                current_k_mdp_state.new_state(action, state_to, reward);
                was_set = true;
                DEBUG_OUT(2,"CHOOSE");
            }
        }
    }
    if(!was_set) {
        DEBUG_OUT(0, "Error: Unnormalized probabilities [sum(p)=" << prob_accum << "]--> no transition performed." );
    }

    DEBUG_OUT(1, "(" << old_state.x() << "," << old_state.y() << ") + " << Data::action_strings[action] << " ==> (" << current_state.x() << "," << current_state.y() << ")");
}


void Maze::perform_transition(const action_t& a, Data::state_t& final_state, reward_t& r) {
    perform_transition(a);
    final_state = current_state.state_idx();
    r = current_k_mdp_state.get_k_mdp_state()[0].reward;
}

void Maze::initialize_predictions(QIteration& predictions) {
    unsigned long counter = 0;
    for(Data::k_mdp_state_idx_t k_mdp_state_idx_from=0; k_mdp_state_idx_from<(idx_t)Data::k_mdp_state_n; ++k_mdp_state_idx_from) {
        Data::k_mdp_state_t k_mdp_state_from = Data::k_mdp_state_from_idx(k_mdp_state_idx_from);
        for(Data::action_idx_t action_idx = 0; action_idx<(idx_t)Data::action_n; ++action_idx) {
            action_t action = Data::action_from_idx(action_idx);
            for(Data::state_idx_t state_to_idx=0; state_to_idx<(idx_t)Data::state_n; ++state_to_idx) {
                state_t state_to = Data::state_from_idx(state_to_idx);
                for(Data::reward_idx_t reward_idx=0; reward_idx<(idx_t)Data::reward_n; ++reward_idx) {
                    reward_t reward = Data::reward_from_idx(reward_idx);
                    predictions.set_prediction(
                            k_mdp_state_from,
                            action,
                            state_to,
                            reward,
                            get_prediction(k_mdp_state_from, action, state_to, reward)
                    );
                    ++counter;
                }
            }
        }
    }
    DEBUG_OUT(1,"Initialized " << counter << " predictions");
}

Maze::probability_t Maze::get_prediction(const k_mdp_state_t& k_mdp_state_from, const action_t& action, const state_t& state_to, const reward_t& reward) const {

    // check for matching reward
//    if(k_mdp_state_from[time_delay-1].state==button_state.state_idx()
//            && state_to==smiley_state.state_idx()
//            && reward!=Data::max_reward ) {
//        return 0;
//    } else if( ( k_mdp_state_from[time_delay-1].state!=button_state.state_idx() || state_to!=smiley_state.state_idx() )
//            && reward==Data::max_reward ) {
//        return 0;
//    }

    // check for matching reward
    reward_t necessary_reward = Data::min_reward;
    for(idx_t idx=0; idx<(idx_t)rewards_n; ++idx) {
        state_t activate_state = Data::state_from_idx(rewards[idx][0]);
        state_t receive_state = Data::state_from_idx(rewards[idx][1]);
        state_t state_back_then = k_mdp_state_from[rewards[idx][2]-1].state;
        state_t state_now = state_to;
        if( activate_state==state_back_then && receive_state==state_now) {
            necessary_reward = Data::max_reward;
        }
    }
    if(necessary_reward!=reward) return 0;

    // check for matching state
    MazeState maze_state_to( state_to );
    MazeState state_from( k_mdp_state_from[0].state);
    MazeState state_left( clamp(0,Data::maze_x_dim-1,state_from.x()-1),clamp(0,Data::maze_y_dim-1,state_from.y()  ));
    MazeState state_right(clamp(0,Data::maze_x_dim-1,state_from.x()+1),clamp(0,Data::maze_y_dim-1,state_from.y()  ));
    MazeState state_up(   clamp(0,Data::maze_x_dim-1,state_from.x()  ),clamp(0,Data::maze_y_dim-1,state_from.y()-1));
    MazeState state_down( clamp(0,Data::maze_x_dim-1,state_from.x()  ),clamp(0,Data::maze_y_dim-1,state_from.y()+1));

    // consider walls
    for(idx_t idx=0; idx<(idx_t)walls_n; ++idx) {
        MazeState maze_state_1(Data::state_from_idx(walls[idx][0]));
        MazeState maze_state_2(Data::state_from_idx(walls[idx][1]));
        if( state_from == maze_state_1 ) {
            if(state_left  == maze_state_2) state_left  = state_from;
            if(state_right == maze_state_2) state_right = state_from;
            if(state_up    == maze_state_2) state_up    = state_from;
            if(state_down  == maze_state_2) state_down  = state_from;
        } else if(state_from == maze_state_2) {
            if(state_left  == maze_state_1) state_left  = state_from;
            if(state_right == maze_state_1) state_right = state_from;
            if(state_up    == maze_state_1) state_up    = state_from;
            if(state_down  == maze_state_1) state_down  = state_from;
        }
    }

    probability_t prob = 0;

    if(maze_state_to==state_from ) prob += epsilon/5;
    if(maze_state_to==state_left ) prob += epsilon/5;
    if(maze_state_to==state_right) prob += epsilon/5;
    if(maze_state_to==state_up   ) prob += epsilon/5;
    if(maze_state_to==state_down ) prob += epsilon/5;

    switch(action) {
    case Data::STAY:
        if(maze_state_to==state_from ) prob += 1-epsilon;
        break;
    case Data::LEFT:
        if(maze_state_to==state_left ) prob += 1-epsilon;
        break;
    case Data::RIGHT:
        if(maze_state_to==state_right) prob += 1-epsilon;
        break;
    case Data::UP:
        if(maze_state_to==state_up   ) prob += 1-epsilon;
        break;
    case Data::DOWN:
        if(maze_state_to==state_down ) prob += 1-epsilon;
        break;
    default:
        DEBUG_OUT(0,"Error: unknown action");
        return 0;
    }

    return prob;
}

void Maze::set_time_delay(const int& new_time_delay) {
    if(new_time_delay>(idx_t)Data::k) {
        DEBUG_OUT(0,"Error: Reward time delays larger than history length 'k' not allowed");
    } else if(new_time_delay<0){
        DEBUG_OUT(0,"Error: Time delay must be larger than zero --> keeping old value");
    } else {
        time_delay = new_time_delay;
    }
}

void Maze::set_epsilon(const double& e) {
    epsilon = e;
}

void Maze::set_current_state(const state_t& state) {
    current_state = MazeState(state);
    for(idx_t k_idx=0; k_idx<(idx_t)Data::k; ++k_idx) current_k_mdp_state.new_state(Data::STAY, current_state.state_idx(), Data::min_reward);
    DEBUG_OUT(1,"Set current state to (" << current_state.x() << "," << current_state.y() << ")");
}

void Maze::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}
