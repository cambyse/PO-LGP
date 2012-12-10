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

#define DEBUG_LEVEL 2
#include "debug.h"

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

    template< class Predictions >
    void initialize_predictions(Predictions&);

    void set_time_delay(const int& new_time_delay);
    int get_time_delay() { return time_delay; }

    void set_epsilong(const double& e);
    double get_epsilong() const { return epsilon; }

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

template< class Predictions >
void Maze::initialize_predictions(Predictions& predictions) {
    if(time_delay<=0) {
        DEBUG_OUT(0,"Error: Time delay must be larger than zero (is " << time_delay << ")");
        return;
    }
    unsigned long counter = 0;
    for(Data::k_mdp_state_idx_t state_from=0; state_from<Data::k_mdp_state_n; ++state_from) {
        for(Data::action_t action = 0; action<Data::action_n; ++action) {

            Data::k_mdp_state_t k_mdp_state_from = Data::k_mdp_state_from_idx(state_from);

            MazeState maze_state_from(k_mdp_state_from[0].state);
            std::vector<std::tuple<MazeState,probability_t> > state_vector = transition_map[std::make_tuple(maze_state_from,action)];

            for(uint idx=0; idx<state_vector.size(); ++idx) {
                state_t state_to = std::get<0>(state_vector[idx]).state_idx();

                reward_t reward;
                if(k_mdp_state_from[time_delay-1].state==button_state.state_idx()) {
                    if(state_to==smiley_state.state_idx()) {
                        reward = Data::max_reward;
                    } else {
                        reward = Data::min_reward;
                    }
                } else {
                    reward = Data::min_reward;
                }

                predictions.set_prediction(
                        k_mdp_state_from,
                        action,
                        state_to,
                        reward,
                        std::get<1>(state_vector[idx])
                );

                ++counter;
            }

        }
    }
    DEBUG_OUT(1,"Initialized " << counter << " predictions");
}

#include "debug_exclude.h"

#endif /* MAZE_H_ */
