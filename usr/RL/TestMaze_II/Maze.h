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
        int idx() const { return index; }
        int x() const { return index%Data::maze_x_dim; }
        int y() const { return index/Data::maze_x_dim; }
    private:
        int index;
    };

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view);

    void perform_transition(const action_t& action);
    void perform_transition(const action_t& a, Data::state_t& final_state, reward_t& r );

    template< class TransitionProbabilities >
    void initialize_transition_probabilities(TransitionProbabilities&);

    template< class ExpectedRewards >
    void initialize_expected_rewards(ExpectedRewards&);

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

template< class TransitionProbabilities >
void Maze::initialize_transition_probabilities(TransitionProbabilities& transition_probabilities) {
    for(Data::state_t state_from=0; state_from<Data::state_n; ++state_from) {
        for(Data::action_t action = 0; action<Data::action_n; ++action) {

            std::vector<std::tuple<MazeState,probability_t> > state_vector = transition_map[std::make_tuple(state_from,action)];

            for(uint idx=0; idx<state_vector.size(); ++idx) {
                transition_probabilities.set_transition_probability(
                        state_from,
                        action,
                        std::get<0>(state_vector[idx]).idx(),
                        std::get<1>(state_vector[idx])
                );
            }

        }
    }
}

template< class ExpectedRewards >
void Maze::initialize_expected_rewards(ExpectedRewards& expected_rewards) {
    if(time_delay<=0) { // check for zero delay
        // reward would depend on state_to and not on k-MDP state
        DEBUG_OUT(0, "Error: A reward delay smaller of equal zero is not handled correctly.");
        return;
    } else {
        for(Data::k_mdp_state_idx_t k_mdp_state_idx=0; k_mdp_state_idx<Data::k_mdp_state_n; ++k_mdp_state_idx) {
            Data::k_mdp_state_t k_mdp_state = Data::k_mdp_state_from_idx(k_mdp_state_idx);
            if(k_mdp_state.size()==0) { // check for zero size k-MDP states
                DEBUG_OUT(0, "Error: Zero-size k-MDP states are not handled correctly.");
                return;
            } else {
                state_t state_from = k_mdp_state.back().state;
                for(action_t action=0; action<Data::action_n; ++action) {
                    std::vector< std::tuple<MazeState,probability_t> > state_vector = transition_map[std::make_tuple(state_from,action)];
                    for(uint state_vector_idx=0; state_vector_idx<state_vector.size(); ++state_vector_idx) {
                        state_t state_to = std::get<0>(state_vector[state_vector_idx]).idx();
                        probability_t prob = std::get<1>(state_vector[state_vector_idx]);
                        if(MazeState(state_to)==smiley_state && prob!=0) { // finite probability for smiley state
                            // Rewards are deterministic, so no expectation value needs to be computed.
                            if(MazeState(k_mdp_state[k_mdp_state.size()-time_delay].state)==button_state) { // visited button state
                                expected_rewards.set_expected_reward(k_mdp_state,action,state_to,Data::max_reward);
                            } else {
                                expected_rewards.set_expected_reward(k_mdp_state,action,state_to,Data::min_reward);
                            }
                        }
                    }
                }
            }
        }
    }
}

#include "debug_exclude.h"

#endif /* MAZE_H_ */
