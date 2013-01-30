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
#include "KMDPState.h"

#include "debug.h"

class Maze {
public:

    typedef Data::action_t        action_t;
    typedef Data::state_t         state_t;
    typedef Data::reward_t        reward_t;
    typedef Data::probability_t   probability_t;
    typedef Data::input_data_t    input_data_t;
    typedef Data::output_data_t   output_data_t;
    typedef Data::k_mdp_state_t   k_mdp_state_t;
    typedef Data::idx_t           idx_t;
    typedef Data::size_t          size_t;

    enum VALIDATION_TYPE { EXACT_VALIDATION, MONTE_CARLO_VALIDATION };

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
    probability_t get_prediction(const k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const;
    probability_t (Maze::*get_prediction_ptr())(const k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const {
        return &Maze::get_prediction;
    }

    template < class Model >
    probability_t validate_model(
            const Model& model,
            probability_t(Model::*prediction)(const Data::k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const,
            VALIDATION_TYPE type,
            size_t samples = 0
    );

    void set_time_delay(const int& new_time_delay);
    int get_time_delay() { return time_delay; }

    void set_epsilon(const double& e);
    double get_epsilon() const { return epsilon; }

    void set_current_state(const state_t&);

private:

    int time_delay;
    bool reward_active;
    KMDPState current_k_mdp_state;
    static const double state_size;
    double epsilon;
    MazeState current_state;
    MazeState button_state, smiley_state;
    QGraphicsSvgItem *agent, *button, *smiley;

    /*! \brief Rescale the scene to fit into view. */
    void rescale_scene(QGraphicsView * view);

    static int clamp(const int& lower, const int& upper, const int& value) {
        if(value<lower) {
            return lower;
        } else if(value>upper) {
            return upper;
        }
        return value;
    }
};

template < class Model >
Maze::probability_t Maze::validate_model(
        const Model& model,
        probability_t(Model::*prediction)(const Data::k_mdp_state_t&, const action_t&, const state_t&, const reward_t&) const,
        VALIDATION_TYPE type,
        size_t samples
) {
    probability_t kl_divergence = 0;
    switch(type) {
    case MONTE_CARLO_VALIDATION:
        for(size_t transition_counter=0; transition_counter<samples; ++transition_counter) {
            k_mdp_state_t k_mdp_state = current_k_mdp_state.get_k_mdp_state();
            action_t action = rand()%Data::NUMBER_OF_ACTIONS;
            state_t state;
            reward_t reward;
            perform_transition(action,state,reward);
            probability_t p_maze = get_prediction(k_mdp_state,action,state,reward);
            probability_t p_model = (model.*prediction)(k_mdp_state,action,state,reward);
            kl_divergence += log(p_maze/p_model);
        }
        return kl_divergence/samples;
    case EXACT_VALIDATION:
        for(Data::k_mdp_state_idx_t k_mdp_state_idx=0; k_mdp_state_idx<Data::k_mdp_state_n; ++k_mdp_state_idx) {
            k_mdp_state_t k_mdp_state = Data::k_mdp_state_from_idx(k_mdp_state_idx);
            for(Data::action_idx_t action_idx=0; action_idx<Data::action_n; ++action_idx) {
                action_t action = Data::action_from_idx(action_idx);
                for(Data::state_idx_t state_idx=0; state_idx<Data::state_n; ++state_idx) {
                    state_t state = Data::state_from_idx(state_idx);
                    for(Data::reward_idx_t reward_idx=0; reward_idx<Data::reward_n; ++reward_idx) {
                        reward_t reward = Data::reward_from_idx(reward_idx);
                        probability_t p_maze = get_prediction(k_mdp_state,action,state,reward);
                        probability_t p_model = (model.*prediction)(k_mdp_state,action,state,reward);
                        if(p_maze>0) {
                            kl_divergence += p_maze*log(p_maze/p_model);
                        }
                    }
                }
            }
        }
        return kl_divergence;
    }
    DEBUG_OUT(0,"Error: Validation method not handled");
    return 0;
}

#include "debug_exclude.h"

#endif /* MAZE_H_ */
