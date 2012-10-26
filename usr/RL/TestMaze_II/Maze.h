/*
 * Maze.h
 *
 *  Created on: Oct 10, 2012
 *      Author: robert
 */

#ifndef MAZE_H_
#define MAZE_H_

#include <QtGui/QGraphicsView>
#include <QtSvg/QGraphicsSvgItem>
#include <map>
#include <tuple>
#include <deque>

#define DEBUG_LEVEL 0
#include "debug.h"

class Maze_default_renderer {
//    friend class Maze;
public:
    void render_initialize() {}
    void render_update() {}
};

template< class Renderer = Maze_default_renderer >
class Maze {
public:

    enum ACTION { STAY, UP, DOWN, LEFT, RIGHT, NUMBER_OF_ACTIONS };
    typedef ACTION action_t;
    typedef int state_idx;
    typedef std::tuple<state_idx,state_idx> state_t;
    typedef double reward_t;

    Maze(const int& x_dimension, const int& y_dimension, const int& td = 1, Renderer r = Renderer());

    virtual ~Maze();

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view);

    void perform_transition(const action_t& a);
    void perform_transition(const action_t& a, state_t& final_state, reward_t& r );

    void set_time_delay(const int& new_time_delay);

private:

    int x_dim; ///< x dimension.
    int y_dim; ///< y dimension.
    int time_delay;
    std::deque<bool> reward_timer;
    Renderer renderer;
    state_t current_state;
    std::map< std::tuple<state_t,action_t>, std::tuple<state_t,double> > transition_map;
    state_t button_state, smiley_state;
    QGraphicsSvgItem *agent, *button, *smiley;

    void set_current_state(const state_t& s);

    /*! \brief Rescale the scene to fit into view. */
    void rescale_scene(QGraphicsView * view);

    /*! \brief Initialize all transitions. */
    void create_transitions(const double& random);

};

template< class Renderer>
Maze<Renderer>::Maze(const int& x_dimension, const int& y_dimension, const int& td, Renderer r):
        x_dim(x_dimension), y_dim(y_dimension),
        time_delay(td), reward_timer(),
        renderer(r),
        agent(NULL), button(NULL), smiley(NULL) {

    // initializing transitions
    create_transitions(0);

    // setting button and smiley state
    if(x_dim>0 || y_dim>0) {
        button_state = std::make_pair(x_dim-1, y_dim-1);
    } else {
        button_state = std::make_pair(0,0);
    }
    smiley_state = std::make_pair(0,0);

    // setting current state
    current_state = std::make_tuple(x_dim/2,y_dim/2);

    // initialize reward timer and update reward function
    for(int t=0; t<time_delay; ++t) { reward_timer.push_back(false); }
    reward_timer.push_back(current_state==button_state);
}

template< class Renderer>
Maze<Renderer>::~Maze() {
    delete agent;
    delete button;
    delete smiley;
}

template< class Renderer>
void Maze<Renderer>::render_initialize(QGraphicsView * view) {
    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();
    if(scene==NULL) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

//    QPen wall_pen(QBrush(QColor(0,0,0)),0.1,Qt::SolidLine,Qt::SquareCap,Qt::MiterJoin);
//    QPen action_pen(QBrush(QColor(0,0,0,100)),0.06,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin);
//    QPen max_action_pen(QBrush(QColor(0,0,0)),0.06,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin);
    for(state_idx x_idx=0; x_idx<x_dim; ++x_idx) {
        for(state_idx y_idx=0; y_idx<x_dim; ++y_idx) {
            scene->addRect( x_idx, y_idx, 0.9, 0.9, QPen(QColor(0,0,0)), QBrush(QColor(230,230,230)) );
        }
    }

    // initialize and render button state
    if(!button) {
        button = new QGraphicsSvgItem("./button.svg");
        button->setScale(0.3);
        button->setPos(std::get<0>(button_state), std::get<1>(button_state));
    }
    button->setElementId(current_state==button_state ? "active" : "passive");
    scene->addItem(button);

    // initialize and render smiley state
    if(!smiley) {
        smiley = new QGraphicsSvgItem("./smiley.svg");
        smiley->setScale(0.3);
        smiley->setPos(std::get<0>(smiley_state), std::get<1>(smiley_state));
    }
    smiley->setElementId(reward_timer.front() ? "active" : "passive");
    scene->addItem(smiley);

    // initialize Renderer
    renderer.render_initialize();

    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("./agent.svg");
        agent->setScale(0.3);
    }
    agent->setPos(std::get<0>(current_state), std::get<1>(current_state));
    scene->addItem(agent);
//    scene->addEllipse( std::get<0>(current_state) + 0.1, std::get<1>(current_state)+ 0.1, 0.7, 0.7, QPen(QColor(0,0,0)), QBrush(QColor(0,230,0)) );

    rescale_scene(view);
}

template< class Renderer>
void Maze<Renderer>::render_update(QGraphicsView * view) {
    button->setElementId(current_state==button_state ? "active" : "passive");
    smiley->setElementId(reward_timer.front() ? "active" : "passive");
    renderer.render_update();
    agent->setPos(std::get<0>(current_state), std::get<1>(current_state));
    rescale_scene(view);
}

template< class Renderer>
void Maze<Renderer>::perform_transition(const action_t& a) {
    state_t old_state = current_state;
    current_state = std::get<0>(transition_map[std::make_tuple(current_state,a)]); // todo include transition probabilities, deterministic so far.
    reward_timer.pop_front(); // pop old reward
    reward_timer.push_back(current_state==button_state); // push new reward

    if(DEBUG_LEVEL>0) {
        const char * action_name;
        switch(a) {
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
        DEBUG_OUT(1, "(" << std::get<0>(old_state) << "," << std::get<1>(old_state) << ") + " << action_name << " ==> (" << std::get<0>(current_state) << "," << std::get<1>(current_state) << ")");
    }
}

template< class Renderer>
void Maze<Renderer>::perform_transition(const action_t& a, state_t& final_state, reward_t& r) {
    perform_transition(a);
    final_state = current_state;
    r = reward_timer.front() && current_state==smiley_state ? 1 : 0;
}

template< class Renderer>
void Maze<Renderer>::set_time_delay(const int& new_time_delay) {
    if( new_time_delay > time_delay ) {
        for(int t=time_delay; t<=new_time_delay; ++t) reward_timer.push_back(false); // rewards are NOT being "rescheduled"
//        for(int t=time_delay; t<=new_time_delay; ++t) reward_timer.push_front(false); // rewards ARE being "rescheduled"
    } else if( new_time_delay < time_delay ) {
        for(int t=time_delay; t>=new_time_delay; --t) reward_timer.pop_back(); // too old reward memory is lost
    }
    time_delay = new_time_delay;
}

template< class Renderer>
void Maze<Renderer>::set_current_state(const state_t& s) {
    current_state = s;
}

template< class Renderer>
void Maze<Renderer>::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}

template< class Renderer>
void Maze<Renderer>::create_transitions(const double& random) {
    for(state_idx x_idx=0; x_idx<x_dim; ++x_idx) {
        for(state_idx y_idx=0; y_idx<x_dim; ++y_idx) {

            // intentional stay
            transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),STAY)]=std::make_tuple( std::make_tuple(x_idx,y_idx), 1-random); // stay

            // left
            if(x_idx>0) {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),LEFT)]=std::make_tuple( std::make_tuple(x_idx-1,y_idx), 1-random); // left
            } else {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),LEFT)]=std::make_tuple( std::make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

            // right
            if(x_idx<x_dim-1) {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),RIGHT)]=std::make_tuple( std::make_tuple(x_idx+1,y_idx), 1-random); // right
            } else {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),RIGHT)]=std::make_tuple( std::make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

            // up
            if(y_idx>0) {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),UP)]=std::make_tuple( std::make_tuple(x_idx,y_idx-1), 1-random); // up
            } else {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),UP)]=std::make_tuple( std::make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

            // down
            if(y_idx<y_dim-1) {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),DOWN)]=std::make_tuple( std::make_tuple(x_idx,y_idx+1), 1-random); // down
            } else {
                transition_map[std::make_tuple(std::make_tuple(x_idx,y_idx),DOWN)]=std::make_tuple( std::make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

        }
    }
}

#include "debug_exclude.h"

#endif /* MAZE_H_ */
