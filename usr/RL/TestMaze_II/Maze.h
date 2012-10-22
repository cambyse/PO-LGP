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

class Maze
{

public:

    enum ACTION { STAY, UP, DOWN, LEFT, RIGHT, NUMBER_OF_ACTIONS };
    typedef ACTION action;
    typedef int state_idx;
    typedef std::tuple<state_idx,state_idx> state;

    Maze(const int& x_dimension, const int& y_dimension);

    virtual ~Maze();

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view);

    void perform_transition(const action& a);

    void set_current_state(const state& s);

private:

    int x_dim; ///< x dimension.
    int y_dim; ///< y dimension.
    state current_state;
    std::map< std::tuple<state,action>, std::tuple<state,double> > transition_map;

    QGraphicsSvgItem *agent, *button, *smiley;

    /*! \brief Rescale the scene to fit into view. */
    void rescale_scene(QGraphicsView * view);

    /*! \brief Initialize all transitions. */
    void create_transitions(const double& random);

};

#endif /* MAZE_H_ */
