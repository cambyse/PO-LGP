
#include "Maze.h"
#include "util.h"

#define DEBUG_LEVEL 0
#include "debug.h"

const double Maze::state_size = 0.9;
const double Maze::wall_width = 0.05;
const double Maze::reward_start_size = 0.1;
const double Maze::reward_end_size = 0.2;
const double Maze::reward_end_ratio = 0.5;
const double Maze::text_scale = 0.01;
const double Maze::text_center = 0.3;

using util::min;
using util::max;
using util::INVALID;

const Maze::idx_t Maze::walls[walls_n][2] = {
    /* 2x2 Maze *
    { 0, 1}
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

    /* 4x4 Maze *
    { 6, 7},
    {12,13},
    { 4, 5},
    { 6,10},
    {10,11}
    /**/
};

const Maze::idx_t Maze::rewards[rewards_n][8] = {
    /* 2x2 Maze */
    { 0, 3, 4, 5, ON_RELEASE, 200,   0,   0},
    { 3, 0, 4, 5, ON_RELEASE, 200, 200,   0},
    { 0, 1, 1, 1, ON_RELEASE,   0, 200,   0},
    { 3, 2, 1, 1, ON_RELEASE,   0,   0, 200}
    /**/

    /* 3x3 Maze *
    { 3, 5, 4, 8, ON_RELEASE,   0, 200,   0},
    { 5, 3, 6, 8, ON_RELEASE,   0, 200, 200},
    { 4, 1, 1, 1, ON_RELEASE, 200, 200,   0},
    { 4, 7, 1, 1, ON_RELEASE, 200,   0,   0}
    /**/

    /* 4x4 Maze *
    {  4,  2,  3, 1, ON_RELEASE, 200,   0,   0},
    {  6,  7,  3, 1, ON_RELEASE, 200, 200,   0},
    { 11, 14,  2, 1, ON_RELEASE,   0, 200,   0},
    { 13,  8,  2, 1, ON_RELEASE,   0, 200, 200},
    {  8,  1,  3, 1, ON_RELEASE,   0,   0, 200}
    //{  8,  4,  4, 1, ON_RELEASE, 200,   0, 200}
    /**/
};

Maze::Maze(const double& eps):
        time_delay(Data::k),
        current_instance(nullptr),
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
//    if(Data::maze_x_size>0 || Data::maze_y_size>0) {
//        button_state = MazeState(Data::maze_x_size-1,Data::maze_y_size-1);
//    } else {
//        button_state = MazeState(0,0);
//    }
//    smiley_state = MazeState(0,0);

    // setting current state
    current_instance = instance_t::create(action_t::STAY, current_state.state_idx(), reward_t::min_reward);
    current_state = MazeState(Data::maze_x_size/2, Data::maze_y_size/2);
    DEBUG_OUT(1,"Current Maze State: " << current_state << " (Index: " << current_state.state_idx() << ")" );
    set_current_state(current_state.state_idx());
}


Maze::~Maze() {
    delete agent;
    delete current_instance;
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

    // Render States
    QPen state_pen(QColor(0,0,0), 0.02, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
        MazeState maze_state(state);
        scene->addRect( maze_state.x()-state_size/2, maze_state.y()-state_size/2, state_size, state_size, state_pen, QBrush(QColor(230,230,230)) );
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

    // Render Walls
    QPen wall_pen(QColor(50,50,50), 0.02, Qt::SolidLine, Qt::RoundCap);
    QBrush wall_brush(QColor(50,50,50));
    for(idx_t idx=0; idx<(idx_t)walls_n; ++idx) {
        MazeState maze_state_1(walls[idx][0]);
        MazeState maze_state_2(walls[idx][1]);
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

    // rewards
    for(idx_t idx=0; idx<(idx_t)rewards_n; ++idx) {
        MazeState maze_state_1(rewards[idx][ACTIVATION_STATE]);
        MazeState maze_state_2(rewards[idx][RECEIVE_STATE]);
        double x_start   = maze_state_1.x();
        double y_start   = maze_state_1.y();
        double x_end     = maze_state_2.x();
        double y_end     = maze_state_2.y();
        double x_shift   = -(y_end-y_start)/5;
        double y_shift   = (x_end-x_start)/5;
        double x_control = (x_start+x_end)/2+x_shift;
        double y_control = (y_start+y_end)/2+y_shift;
        QColor color(   rewards[idx][R],rewards[idx][G],rewards[idx][B]);
        QPen   arc_pen( color,0.02,Qt::SolidLine,Qt::RoundCap );
        QPen   end_pen( color,0.02,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin );
        QBrush brush(   color );

        // arc
        QPainterPath arc_path;
        arc_path.moveTo(x_start, y_start);
        arc_path.quadTo( x_control, y_control, x_end, y_end );
        scene->addPath(arc_path,arc_pen,QBrush(QColor(0,0,0,0)));

        // start
        scene->addEllipse(
                x_start-reward_start_size/2,
                y_start-reward_start_size/2,
                reward_start_size,
                reward_start_size,
                arc_pen,
                brush
        );

        // end
        double end_vector_x = x_end-x_control;
        double end_vector_y = y_end-y_control;
        double end_vector_norm = sqrt( pow(end_vector_x,2) + pow(end_vector_y,2) );
        end_vector_x /= end_vector_norm;
        end_vector_y /= end_vector_norm;
        double end_cross_vector_x = +end_vector_y;
        double end_cross_vector_y = -end_vector_x;
        QPainterPath end_path;
        end_path.moveTo(x_end, y_end);
        end_path.lineTo(
                x_end-reward_end_size*(end_vector_x+end_cross_vector_x*reward_end_ratio/2),
                y_end-reward_end_size*(end_vector_y+end_cross_vector_y*reward_end_ratio/2)
        );
        end_path.lineTo(
                x_end-reward_end_size*(end_vector_x-end_cross_vector_x*reward_end_ratio/2),
                y_end-reward_end_size*(end_vector_y-end_cross_vector_y*reward_end_ratio/2)
        );
        end_path.lineTo(x_end, y_end);
        scene->addPath(end_path,end_pen,QBrush(color));

        // text
        double mid_point_x = x_start + (x_end - x_start)/2;
        double mid_point_y = y_start + (y_end - y_start)/2;
        size_t time_delay = rewards[idx][TIME_DELAY];
        reward_t reward = reward_t::min_reward+reward_t::reward_increment*rewards[idx][REWARD_IDX];
        QGraphicsTextItem * txt = scene->addText(
                QString("t=%1,r=%2").arg(QString::number(time_delay)).arg(QString::number(reward)),
                QFont("",12)
        );
        QRectF box = txt->boundingRect();
        txt->setPos(
                x_control+(mid_point_x-x_control)*text_center-text_scale*box.width()/2,
                y_control+(mid_point_y-y_control)*text_center-text_scale*box.height()/2
        );
        txt->setScale(text_scale);
        txt->setDefaultTextColor(color);
    }

    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("./agent.svg");
        agent->setScale(0.2);
        QSizeF s = agent->boundingRect().size();
        agent->setPos(current_state.x()-s.width()/2, current_state.y()-s.height()/2);
        agent->setElementId("normal");
    }

    scene->addItem(agent);

    rescale_scene(view);
}


void Maze::render_update(QGraphicsView * view) {
//    button->setElementId(current_state==button_state ? "active" : "passive");
//    smiley->setElementId( reward_active ? "active" : "passive");
    QSizeF s = agent->boundingRect().size();
    agent->setPos(current_state.x()-agent->scale()*s.width()/2, current_state.y()-agent->scale()*s.height()/2);
    agent->setElementId(agent->elementId()=="normal" ? "mirrored" : "normal");
    rescale_scene(view);
}


void Maze::perform_transition(const action_t& action) {
    MazeState old_state = current_state; // remember current (old) state

    DEBUG_OUT(1,"Current instance: ");
    const_instanceIt_t insIt = current_instance->it();
    for(idx_t k_idx=0; k_idx<(idx_t)Data::k; ++k_idx) {
        DEBUG_OUT(1,"    " << (*insIt) );
        --insIt;
    }

    // perform transition
    probability_t prob_threshold = drand48();
    DEBUG_OUT(2,"Prob threshold = " << prob_threshold);
    probability_t prob_accum = 0;
    bool was_set = false;
    for(stateIt_t state_to=stateIt_t::first(); state_to!=INVALID && !was_set; ++state_to) {
        for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID && !was_set; ++reward) {
            probability_t prob = get_prediction(current_instance, action, state_to, reward);
            DEBUG_OUT(2,"state(" << state_to << "), reward(" << reward << ") --> prob=" << prob);
            prob_accum += prob;
            if(prob_accum>prob_threshold) {
                current_state = MazeState(state_to);
                current_instance = current_instance->append_instance(action, state_to, reward);
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


void Maze::perform_transition(const action_t& a, state_t& final_state, reward_t& r) {
    perform_transition(a);
    final_state = current_state.state_idx();
    r = current_instance->reward;
}

Maze::probability_t Maze::get_prediction(const instance_t* instance_from, const action_t& action, const state_t& state_to, const reward_t& reward) const {

    // check for matching reward
//    if(instance_from[time_delay-1].state==button_state.state_idx()
//            && state_to==smiley_state.state_idx()
//            && reward!=reward_t::max_reward ) {
//        return 0;
//    } else if( ( instance_from[time_delay-1].state!=button_state.state_idx() || state_to!=smiley_state.state_idx() )
//            && reward==reward_t::max_reward ) {
//        return 0;
//    }

    DEBUG_OUT(2,"Getting prediction for:");
    DEBUG_OUT(2,"    Action: " << action);
    DEBUG_OUT(2,"     State: " << state_to);
    DEBUG_OUT(2,"    Reward: " << reward);
    DEBUG_OUT(2,"    ------------------------------");

    // check for matching reward
    reward_t matching_reward = reward_t::min_reward;
    for(idx_t idx=0; idx<(idx_t)rewards_n; ++idx) {
        state_t activate_state = rewards[idx][ACTIVATION_STATE];
        state_t receive_state = rewards[idx][RECEIVE_STATE];
        state_t state_back_then = (instance_from->const_it() - (rewards[idx][TIME_DELAY]-1))->state;
        state_t state_now = state_to;
        reward_t single_reward = reward_t::min_reward+reward_t::reward_increment*rewards[idx][REWARD_IDX];
        DEBUG_OUT(2,"    reward index               :" << idx);
        DEBUG_OUT(2,"    time delay                 :" << rewards[idx][TIME_DELAY]);
        DEBUG_OUT(2,"    activation  (target/actual):" << activate_state << "/" << state_back_then);
        DEBUG_OUT(2,"    reception   (target/actual):" << receive_state << "/" << state_now);
        DEBUG_OUT(2,"    activation type            :" << (rewards[idx][ACTIVATION_TYPE]==EACH_TIME ? "EACH_TIME" : "ON_RELEASE") );
        DEBUG_OUT(2,"    reward (single/accumulated):" << single_reward << "/" << reward);
        if( activate_state==state_back_then && receive_state==state_now) {
            bool increment = true;
            if(rewards[idx][ACTIVATION_TYPE]==ON_RELEASE) {
                idx_t delay_counter = 1;
                for(const_instanceIt_t insIt = instance_from->const_it();
                    insIt!=INVALID && delay_counter<rewards[idx][TIME_DELAY];
                    --insIt, ++delay_counter) {
                    if(insIt->state==activate_state) {
                        increment = false;
                        DEBUG_OUT(2,"    --> Activation type (ON_RELEASE) prohibits reward.");
                        break;
                    }
                }
            }
            if(increment) {
                matching_reward += single_reward;
                DEBUG_OUT(2,"    --> Increment matching reward by " << single_reward << " to " << matching_reward);
            }
        }
        DEBUG_OUT(2,"    ------------------------------");
    }

    // crop to max reward
    if(matching_reward>reward_t::max_reward) {
        matching_reward = reward_t::max_reward;
        DEBUG_OUT(2,"    Cropping matching reward to " << matching_reward);
    }
    if(reward!=matching_reward) {
        DEBUG_OUT(2,"    Reward does not match: reward=" << reward << ", should be " << matching_reward);
        return 0;
    }

    // check for matching state
    MazeState maze_state_to( state_to );
    MazeState state_from( instance_from->state);
    MazeState state_left( clamp(0,Data::maze_x_size-1,state_from.x()-1),clamp(0,Data::maze_y_size-1,state_from.y()  ));
    MazeState state_right(clamp(0,Data::maze_x_size-1,state_from.x()+1),clamp(0,Data::maze_y_size-1,state_from.y()  ));
    MazeState state_up(   clamp(0,Data::maze_x_size-1,state_from.x()  ),clamp(0,Data::maze_y_size-1,state_from.y()-1));
    MazeState state_down( clamp(0,Data::maze_x_size-1,state_from.x()  ),clamp(0,Data::maze_y_size-1,state_from.y()+1));

    // consider walls
    for(idx_t idx=0; idx<(idx_t)walls_n; ++idx) {
        MazeState maze_state_1(walls[idx][0]);
        MazeState maze_state_2(walls[idx][1]);
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
    case action_t::STAY:
        if(maze_state_to==state_from ) prob += 1-epsilon;
        break;
    case action_t::LEFT:
        if(maze_state_to==state_left ) prob += 1-epsilon;
        break;
    case action_t::RIGHT:
        if(maze_state_to==state_right) prob += 1-epsilon;
        break;
    case action_t::UP:
        if(maze_state_to==state_up   ) prob += 1-epsilon;
        break;
    case action_t::DOWN:
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
    for(idx_t k_idx=0; k_idx<(idx_t)Data::k; ++k_idx) {
        current_instance = current_instance->append_instance(action_t::STAY, current_state.state_idx(), reward_t::min_reward);
    }
    DEBUG_OUT(1,"Set current state to (" << current_state.x() << "," << current_state.y() << ")");
}

void Maze::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}
