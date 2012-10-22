/*
 * Maze.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: robert
 */

#include "Maze.h"
#define DEBUG_LEVEL 1
#include "debug.h"

using std::make_tuple;
using std::get;

Maze::Maze(const int& x_dimension, const int& y_dimension):
        x_dim(x_dimension), y_dim(y_dimension),
        agent(NULL), button(NULL), smiley(NULL) {
    create_transitions(0);
    current_state = make_tuple(x_dim/2,y_dim/2);
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

//    QPen wall_pen(QBrush(QColor(0,0,0)),0.1,Qt::SolidLine,Qt::SquareCap,Qt::MiterJoin);
//    QPen action_pen(QBrush(QColor(0,0,0,100)),0.06,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin);
//    QPen max_action_pen(QBrush(QColor(0,0,0)),0.06,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin);
    for(state_idx x_idx=0; x_idx<x_dim; ++x_idx) {
        for(state_idx y_idx=0; y_idx<x_dim; ++y_idx) {
            scene->addRect( x_idx, y_idx, 0.9, 0.9, QPen(QColor(0,0,0)), QBrush(QColor(230,230,230)) );
        }
    }

    // render button
    if(!button) {
        button = new QGraphicsSvgItem("./button.svg");
        button->setScale(0.3);
        if(x_dim>0 || y_dim>0) {
            button->setPos(x_dim-1, y_dim-1);
        } else {
            button->setPos(0, 0);
        }
    }
    scene->addItem(button);

    // render smiley
    if(!smiley) {
        smiley = new QGraphicsSvgItem("./smiley.svg");
        smiley->setScale(0.3);
        smiley->setPos(0, 0);
    }
    scene->addItem(smiley);

    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("./agent.svg");
        agent->setScale(0.3);
    }
    agent->setPos(get<0>(current_state), get<1>(current_state));
    scene->addItem(agent);
//    scene->addEllipse( get<0>(current_state) + 0.1, get<1>(current_state)+ 0.1, 0.7, 0.7, QPen(QColor(0,0,0)), QBrush(QColor(0,230,0)) );

    rescale_scene(view);
}

void Maze::render_update(QGraphicsView * view) {
    agent->setPos(get<0>(current_state), get<1>(current_state));
    rescale_scene(view);
}

void Maze::perform_transition(const action& a) {
    state old_state = current_state;
    current_state = get<0>(transition_map[make_tuple(current_state,a)]);
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
    DEBUG_OUT(1, "(" << get<0>(old_state) << "," << get<1>(old_state) << ") + " << action_name << " ==> (" << get<0>(current_state) << "," << get<1>(current_state) << ")");
}

void Maze::set_current_state(const state& s) {
    current_state = s;
}

void Maze::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}

void Maze::create_transitions(const double& random) {
    for(state_idx x_idx=0; x_idx<x_dim; ++x_idx) {
        for(state_idx y_idx=0; y_idx<x_dim; ++y_idx) {

            // intentional stay
            transition_map[make_tuple(make_tuple(x_idx,y_idx),STAY)]=make_tuple( make_tuple(x_idx,y_idx), 1-random); // stay

            // left
            if(x_idx>0) {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),LEFT)]=make_tuple( make_tuple(x_idx-1,y_idx), 1-random); // left
            } else {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),LEFT)]=make_tuple( make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

            // right
            if(x_idx<x_dim-1) {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),RIGHT)]=make_tuple( make_tuple(x_idx+1,y_idx), 1-random); // right
            } else {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),RIGHT)]=make_tuple( make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

            // up
            if(y_idx>0) {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),UP)]=make_tuple( make_tuple(x_idx,y_idx-1), 1-random); // up
            } else {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),UP)]=make_tuple( make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

            // down
            if(y_idx<y_dim-1) {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),DOWN)]=make_tuple( make_tuple(x_idx,y_idx+1), 1-random); // down
            } else {
                transition_map[make_tuple(make_tuple(x_idx,y_idx),DOWN)]=make_tuple( make_tuple(x_idx,y_idx), 1-random); // stay forced
            }

        }
    }
}
