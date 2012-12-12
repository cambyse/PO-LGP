/*
 * Maze.h
 *
 *  Created on: Oct 10, 2012
 *      Author: robert
 */

#ifndef MAZE_H_
#define MAZE_H_

#include <QGraphicsView>
#include <QGraphicsSvgItem>
#include <map>
#include <tuple>
#include <deque>
#include <vector>

#include "Data.h"
#include "QIteration.h"

class Maze {
public:

    typedef Data::action_t        action_t;
    typedef Data::state_t         state_t;
    typedef Data::reward_t        reward_t;
    typedef Data::probability_t   probability_t;
    typedef Data::input_data_t    input_data_t;
    typedef Data::output_data_t   output_data_t;

    Maze(const double& eps = 0);

    virtual ~Maze();

    class MazeState {
    public:
        MazeState(const int& idx = 0): index(idx) {}
        MazeState(const int& x, const int& y): index(x+Data::maze_x_dim*y) {}
        bool operator==(const MazeState& other) const { return this->index==other.index; }
        bool operator!=(const MazeState& other) const { return !((*this)==other); }
        bool operator<(const MazeState& other) const { return this->index<other.index; }
        unsigned long state_idx() const { return index; }
        unsigned long x() const { return index%Data::maze_x_dim; }
        unsigned long y() const { return index/Data::maze_x_dim; }
    private:
        unsigned long index;
    };

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view);

    void perform_transition(const action_t& action);
    void perform_transition(const action_t& a, Data::state_t& final_state, reward_t& r );

    void initialize_predictions(QIteration&);

    void set_time_delay(const int& new_time_delay);
    int get_time_delay() { return time_delay; }

    void set_epsilon(const double& e);
    double get_epsilon() const { return epsilon; }

    void set_current_state(const state_t&);

private:

    int time_delay;
    static const double state_size;
    std::deque<bool> reward_timer;
    double epsilon;
    MazeState current_state;
    std::map< std::tuple<MazeState,action_t>, std::vector<std::tuple<MazeState,probability_t> > > transition_map;
    MazeState button_state, smiley_state;
    QGraphicsSvgItem *agent, *button, *smiley;

    void set_current_state(const MazeState& s);

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

#endif /* MAZE_H_ */
