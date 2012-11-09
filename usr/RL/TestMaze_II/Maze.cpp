
#include "Maze.h"

#define DEBUG_LEVEL 0
#include "debug.h"

const double Maze::state_size = 0.9;

Maze::Maze(const int& x_dimension, const int& y_dimension, const int& td, const double& eps, renderer_t r):
        x_dim(x_dimension), y_dim(y_dimension),
        time_delay(td), reward_timer(),
        epsilon(eps), renderer(r),
        agent(NULL), button(NULL), smiley(NULL) {

    // initializing transitions
    create_transitions();

    // setting button and smiley state
    if(x_dim>0 || y_dim>0) {
        button_state = state_t(x_dim-1,y_dim-1,x_dim);
    } else {
        button_state = state_t(0,0,x_dim);
    }
    smiley_state = state_t(0,0,x_dim);

    // setting current state
    current_state = state_t(x_dim/2,y_dim/2,x_dim);

    // initialize reward timer and update reward function
    for(int t=0; t<time_delay; ++t) { reward_timer.push_front(false); }
    reward_timer.push_front(current_state==button_state);
}


Maze::~Maze() {
    delete agent;
    delete button;
    delete smiley;
}


void Maze::render_initialize(QGraphicsView * view) {
    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();
    if(scene==NULL) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

    for(int idx=0; idx<x_dim*y_dim; ++idx) {
        state_t state(idx);
        scene->addRect( state.x(x_dim)-state_size/2, state.y(x_dim)-state_size/2, state_size, state_size, QPen(QColor(0,0,0)), QBrush(QColor(230,230,230)) );
    }

    // initialize and render button state
    if(!button) {
        button = new QGraphicsSvgItem("./button.svg");
        QSizeF s = button->boundingRect().size();
        button->setTransformOriginPoint(s.width()/2,s.height()/2);
        button->setScale(0.2);
        button->setPos(button_state.x(x_dim)-s.width()/2,button_state.y(x_dim)-s.height()/2);
    }
    button->setElementId(current_state==button_state ? "active" : "passive");
    scene->addItem(button);

    // initialize and render smiley state
    if(!smiley) {
        smiley = new QGraphicsSvgItem("./smiley.svg");
        QSizeF s = smiley->boundingRect().size();
        smiley->setTransformOriginPoint(s.width()/2,s.height()/2);
        smiley->setScale(0.2);
        smiley->setPos(smiley_state.x(x_dim)-s.width()/2, smiley_state.y(x_dim)-s.height()/2);
    }
    smiley->setElementId(reward_timer.back() ? "active" : "passive");
    scene->addItem(smiley);

    // initialize renderer
    renderer.render_initialize();

    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("./agent.svg");
        QSizeF s = agent->boundingRect().size();
        agent->setTransformOriginPoint(s.width()/2,s.height()/2);
        agent->setScale(0.2);
        agent->setPos(current_state.x(x_dim)-s.width()/2, current_state.y(x_dim)-s.height()/2);
    }

    scene->addItem(agent);

    rescale_scene(view);
}


void Maze::render_update(QGraphicsView * view) {
    button->setElementId(current_state==button_state ? "active" : "passive");
    smiley->setElementId(reward_timer.back() ? "active" : "passive");
    renderer.render_update();
    QSizeF s = agent->boundingRect().size();
    agent->setPos(current_state.x(x_dim)-s.width()/2, current_state.y(x_dim)-s.height()/2);
    rescale_scene(view);
}


void Maze::perform_transition(const action_t& action) {
    state_t old_state = current_state; // remember current (old) state
    reward_timer.pop_back(); // pop current (old) reward

    // perform transition
    std::vector< std::tuple<state_t,double> > state_vector = transition_map[std::make_tuple(current_state,action)];
    double r = drand48();
    bool was_set = false;
    DEBUG_OUT(2,"r = " << r);
    for(uint idx=0; idx<state_vector.size(); ++idx) {
        r -= std::get<1>(state_vector[idx]);
        state_t state_to = std::get<0>(state_vector[idx]);
        DEBUG_OUT(2,"r = " << r << ", (" << state_to.x(x_dim) << "," << state_to.y(x_dim) << ")" );
        if(r<0) {
            current_state = state_to;
            was_set = true;
            break;
        }
    }
    if(!was_set) {
        DEBUG_OUT(0, "Error: Unnormalized probabilities --> no transition performed." );
    }

    reward_timer.push_front(current_state==button_state); // push new reward

    if(DEBUG_LEVEL>0) {
        const char * action_name;
        switch(action) {
        case STAY:
            action_name = "STAY";
            break;
        case RIGHT:
            action_name = "RIGHT";
            break;
        case LEFT:
            action_name = "LEFT";
            break;
        case UP:
            action_name = "UP";
            break;
        case DOWN:
            action_name = "DOWN";
            break;
        default:
            action_name = "INVALID_ACTION";
            break;
        }
        DEBUG_OUT(1, "(" << old_state.x(x_dim) << "," << old_state.y(x_dim) << ") + " << action_name << " ==> (" << current_state.x(x_dim) << "," << current_state.y(x_dim) << ")");
    }
}


void Maze::perform_transition(const action_t& a, state_t& final_state, reward_t& r) {
    perform_transition(a);
    final_state = current_state;
    r = reward_timer.back() && current_state==smiley_state ? 1 : 0;
}

void Maze::set_time_delay(const int& new_time_delay) {
    if( new_time_delay > time_delay ) {
        // increase reward memory
        for(int t=time_delay; t<new_time_delay; ++t) {
            reward_timer.push_front(false); // rewards are NOT being "rescheduled"
//            reward_timer.push_front(false); // rewards ARE being "rescheduled"
        }
    } else if( new_time_delay < time_delay ) {
        // decrease reward memory
        for(int t=time_delay; t>new_time_delay; --t) {
            reward_timer.pop_front(); // old reward memory is deleted
        }
    }
    time_delay = new_time_delay;
}


void Maze::set_current_state(const state_t& s) {
    current_state = s;
}


void Maze::set_epsilong(const double& e) {
    epsilon = e;
    create_transitions();
}


void Maze::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}


void Maze::create_transitions() {
    for(int idx=0; idx<x_dim*y_dim; ++idx) {

        // reachable states
        state_t state_from(idx);
        state_t state_left( clamp(0,x_dim-1,state_from.x(x_dim)-1),clamp(0,y_dim-1,state_from.y(x_dim)  ),x_dim);
        state_t state_right(clamp(0,x_dim-1,state_from.x(x_dim)+1),clamp(0,y_dim-1,state_from.y(x_dim)  ),x_dim);
        state_t state_up(   clamp(0,x_dim-1,state_from.x(x_dim)  ),clamp(0,y_dim-1,state_from.y(x_dim)-1),x_dim);
        state_t state_down( clamp(0,x_dim-1,state_from.x(x_dim)  ),clamp(0,y_dim-1,state_from.y(x_dim)+1),x_dim);

        // stay
        std::vector< std::tuple<state_t,double> > stay_vector;
        stay_vector.push_back(std::make_tuple(state_from,1-epsilon));
        stay_vector.push_back(std::make_tuple(state_left,epsilon/4));
        stay_vector.push_back(std::make_tuple(state_right,epsilon/4));
        stay_vector.push_back(std::make_tuple(state_up,epsilon/4));
        stay_vector.push_back(std::make_tuple(state_down,epsilon/4));
        transition_map[std::make_tuple(state_from,STAY)] = stay_vector;

        // left
        std::vector< std::tuple<state_t,double> > left_vector;
        left_vector.push_back(std::make_tuple(state_from,epsilon/4));
        left_vector.push_back(std::make_tuple(state_left,1-epsilon));
        left_vector.push_back(std::make_tuple(state_right,epsilon/4));
        left_vector.push_back(std::make_tuple(state_up,epsilon/4));
        left_vector.push_back(std::make_tuple(state_down,epsilon/4));
        transition_map[std::make_tuple(state_from,LEFT)] = left_vector;

        // right
        std::vector< std::tuple<state_t,double> > right_vector;
        right_vector.push_back(std::make_tuple(state_from,epsilon/4));
        right_vector.push_back(std::make_tuple(state_left,epsilon/4));
        right_vector.push_back(std::make_tuple(state_right,1-epsilon));
        right_vector.push_back(std::make_tuple(state_up,epsilon/4));
        right_vector.push_back(std::make_tuple(state_down,epsilon/4));
        transition_map[std::make_tuple(state_from,RIGHT)] = right_vector;

        // up
        std::vector< std::tuple<state_t,double> > up_vector;
        up_vector.push_back(std::make_tuple(state_from,epsilon/4));
        up_vector.push_back(std::make_tuple(state_left,epsilon/4));
        up_vector.push_back(std::make_tuple(state_right,epsilon/4));
        up_vector.push_back(std::make_tuple(state_up,1-epsilon));
        up_vector.push_back(std::make_tuple(state_down,epsilon/4));
        transition_map[std::make_tuple(state_from,UP)] = up_vector;

        // down
        std::vector< std::tuple<state_t,double> > down_vector;
        down_vector.push_back(std::make_tuple(state_from,epsilon/4));
        down_vector.push_back(std::make_tuple(state_left,epsilon/4));
        down_vector.push_back(std::make_tuple(state_right,epsilon/4));
        down_vector.push_back(std::make_tuple(state_up,epsilon/4));
        down_vector.push_back(std::make_tuple(state_down,1-epsilon));
        transition_map[std::make_tuple(state_from,DOWN)] = down_vector;

        if(DEBUG_LEVEL>=2) {
            DEBUG_OUT(2,"From state (" << state_from.x(x_dim) << "," << state_from.y(x_dim) << "):");
            for(int action=0; action<NUMBER_OF_ACTIONS; ++action) {
                DEBUG_OUT(2,"    given Action = " << action);
                std::vector< std::tuple<state_t,double> > state_vector = transition_map[std::make_tuple(state_from,(action_t)action)];
                for(uint idx=0; idx<state_vector.size(); ++idx) {
                    state_t state_to = std::get<0>(state_vector[idx]);
                    double prob = std::get<1>(state_vector[idx]);
                    DEBUG_OUT(2,"        to state (" << state_to.x(x_dim) << "," << state_to.y(x_dim) << ") with probability " << prob);
                }
            }
        }
    }
}
