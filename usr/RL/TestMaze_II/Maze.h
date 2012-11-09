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
#include <vector>

class Maze_default_renderer {
//    friend class Maze;
public:
    void render_initialize() {}
    void render_update() {}
};

class Maze {
public:

    enum ACTION { STAY, UP, DOWN, LEFT, RIGHT, NUMBER_OF_ACTIONS };
    typedef ACTION action_t;
    typedef double reward_t;
    typedef Maze_default_renderer renderer_t;

    class state_t {
    public:
        state_t(const int& idx = 0): index(idx) {}
        state_t(const int& x, const int& y, const int& x_dim): index(x+x_dim*y) {}
        bool operator==(const state_t& other) const { return this->index==other.index; }
        bool operator!=(const state_t& other) const { return !((*this)==other); }
        bool operator<(const state_t& other) const { return this->index<other.index; }
        int idx() const { return index; }
        int x(const int& x_dim) const { return index%x_dim; }
        int y(const int& x_dim) const { return index/x_dim; }
    private:
        int index;
    };

    Maze(const int& x_dimension, const int& y_dimension, const int& td = 1, const double& eps = 0, renderer_t r = renderer_t());

    virtual ~Maze();

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view);

    void perform_transition(const action_t& action);
    void perform_transition(const action_t& a, state_t& final_state, reward_t& r );

    template< class TransitionModel >
    void initialize_transition_model(TransitionModel& model);

    void set_time_delay(const int& new_time_delay);
    int get_time_delay() { return time_delay; }

    int number_of_states() const { return x_dim*y_dim; }

    void set_epsilong(const double& e);
    double get_epsilong() const { return epsilon; }

private:

    int x_dim; ///< x dimension.
    int y_dim; ///< y dimension.
    int time_delay;
    static const double state_size;
    std::deque<bool> reward_timer;
    double epsilon;
    renderer_t renderer;
    state_t current_state;
    std::map< std::tuple<state_t,action_t>, std::vector<std::tuple<state_t,double> > > transition_map;
    state_t button_state, smiley_state;
    QGraphicsSvgItem *agent, *button, *smiley;

    void set_current_state(const state_t& s);

    /*! \brief Rescale the scene to fit into view. */
    void rescale_scene(QGraphicsView * view);

    /*! \brief Initialize all transitions. */
    void create_transitions();

    int clamp(const int& lower, const int& upper, const int& value) {
        if(value<lower) {
            return lower;
        } else if(value>upper) {
            return upper;
        }
        return value;
    }
};

template< class TransitionModel >
void Maze::initialize_transition_model(TransitionModel& model) {
    for(int state_from=0; state_from<x_dim*y_dim; ++state_from) {
        for(int action = 0; action<NUMBER_OF_ACTIONS; ++action) {
            std::vector<std::tuple<state_t,double> > state_vector = transition_map[std::make_tuple(state_from,(ACTION)action)];
            for(uint idx=0; idx<state_vector.size(); ++idx) {
                model.set_transition_probability(state_t(state_from),(ACTION)action,std::get<0>(state_vector[idx]),std::get<1>(state_vector[idx]));
            }
        }
    }
}

#endif /* MAZE_H_ */
