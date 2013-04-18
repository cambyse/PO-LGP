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
#include <string>
#include <sstream>

#include "Representation/Representation.h"
#include "Data.h"

#include "debug.h"

class Maze {
public:

    typedef Data::idx_t           idx_t;
    typedef Data::size_t          size_t;
    typedef Data::probability_t   probability_t;

    enum VALIDATION_TYPE { EXACT_VALIDATION, MONTE_CARLO_VALIDATION };

    Maze(const double& eps = 0);

    virtual ~Maze();

    class MazeState {
    public:
        MazeState(const int& idx = 0): index(idx) {}
        MazeState(const int& x, const int& y): index(x+Data::maze_x_size*y) {}
        bool operator==(const MazeState& other) const { return this->index==other.index; }
        bool operator!=(const MazeState& other) const { return !((*this)==other); }
        bool operator<(const MazeState& other) const { return this->index<other.index; }
        idx_t state_idx() const { return index; }
        idx_t x() const { return index%Data::maze_x_size; }
        idx_t y() const { return index/Data::maze_x_size; }
        std::string print() {
            std::stringstream ss;
            ss << "(" << x() << "," << y() << ")";
            return ss.str();
        }
    private:
        idx_t index;
    };

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view);

    void perform_transition(const action_t& action);
    void perform_transition(const action_t& a, state_t& final_state, reward_t& r );

    probability_t get_prediction(const instance_t&, const action_t&, const state_t&, const reward_t&) const;
    probability_t (Maze::*get_prediction_ptr())(const instance_t&, const action_t&, const state_t&, const reward_t&) const {
        return &Maze::get_prediction;
    }

    template < class Model >
    probability_t validate_model(
            const Model& model,
            probability_t(Model::*prediction)(const instance_t&, const action_t&, const state_t&, const reward_t&) const,
            VALIDATION_TYPE type,
            size_t samples = 0,
            probability_t * mean_model_likelihood = nullptr,
            probability_t * mean_maze_likelihood = nullptr
    );

    void set_time_delay(const int& new_time_delay);
    int get_time_delay() { return time_delay; }

    void set_epsilon(const double& e);
    double get_epsilon() const { return epsilon; }

    void set_current_state(const state_t&);

private:

    int time_delay;
//    bool reward_active;
    instance_t current_instance;
    static const double state_size;
    static const double wall_width;
    static const double reward_start_size;
    static const double reward_end_size;
    static const double reward_end_ratio;
    static const double text_scale;
    static const double text_center;
    double epsilon;
    MazeState current_state;
//    MazeState button_state, smiley_state;
//    QGraphicsSvgItem *button, *smiley;
    QGraphicsSvgItem *agent;

    static const size_t walls_n = 8;
    static const idx_t walls[walls_n][2];

    static const size_t rewards_n = 4;
    static const idx_t rewards[rewards_n][7];
    enum REWARD_COMPONENTS { ACTIVATION_STATE, RECEIVE_STATE, TIME_DELAY, REWARD_IDX, R, G, B };

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
        probability_t(Model::*prediction)(const instance_t&, const action_t&, const state_t&, const reward_t&) const,
        VALIDATION_TYPE type,
        size_t samples,
        probability_t * mean_model_likelihood,
        probability_t * mean_maze_likelihood
) {
    probability_t kl_divergence = 0;
    if(mean_model_likelihood!=nullptr) {
        *mean_model_likelihood = 0;
    }
    if(mean_maze_likelihood!=nullptr) {
        *mean_maze_likelihood = 0;
    }
    switch(type) {
    case MONTE_CARLO_VALIDATION:
        for(size_t transition_counter=0; transition_counter<samples; ++transition_counter) {
            instance_t instance = current_instance;
            action_t action = action_t::random_action();
            state_t state;
            reward_t reward;
            perform_transition(action,state,reward);
            probability_t p_maze = get_prediction(instance,action,state,reward);
            probability_t p_model = (model.*prediction)(instance,action,state,reward);
            kl_divergence += log(p_maze/p_model);
            if(mean_model_likelihood!=nullptr) {
                *mean_model_likelihood += p_model;
            }
            if(mean_maze_likelihood!=nullptr) {
                *mean_maze_likelihood += p_maze;
            }
        }
        if(mean_model_likelihood!=nullptr) {
            *mean_model_likelihood/=samples;
        }
        if(mean_maze_likelihood!=nullptr) {
            *mean_maze_likelihood/=samples;
        }
        return kl_divergence/samples;
    case EXACT_VALIDATION:
        for(instanceIt_t instance=instanceIt_t::first(); instance!=util::INVALID; ++instance) {
            for(actionIt_t action=actionIt_t::first(); action!=util::INVALID; ++action) {
                for(stateIt_t state=stateIt_t::first(); state!=util::INVALID; ++state) {
                    for(rewardIt_t reward=rewardIt_t::first(); reward!=util::INVALID; ++reward) {
                        probability_t p_maze = get_prediction(instance,action,state,reward);
                        probability_t p_model = (model.*prediction)(instance,action,state,reward);
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
