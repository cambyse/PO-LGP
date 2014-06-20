#include "GoalIteration.h"
#include "../util/util.h"
#include <vector>
#include <algorithm> // for std::max
#include "../Representation/DoublyLinkedInstance.h"

#include "../util/debug.h"

using std::vector;
using std::max;
using arma::mat;
using arma::vec;
using arma::zeros;

GoalIteration::GoalIteration(const double & d, const Predictor & predictor, bool auto_it):
    discount(d), auto_iterate(auto_it)
{
    // set spaces and initialize vectors/matrices
    adopt_spaces(predictor);
    int o_size = observation_space->space_size();
    int a_size = action_space->space_size();
    Q = zeros(o_size*a_size);
    R = zeros(o_size);
    V = zeros(o_size);
    p = mat(o_size*a_size,o_size);
    // fill prediction matrix
    int a_o_idx = 0;
    for(auto from_observation : observation_space) {
        for(auto performed_action : action_space) {
            int to_o_idx = 0;
            for(auto to_observation : observation_space) {
                probability_t prob = 0;
                for(auto to_reward : reward_space) {
                    instance_ptr_t i = DoublyLinkedInstance::create(action_space,from_observation,reward_space);
                    prob += predictor.get_prediction(i, performed_action, to_observation, to_reward);
                }
                p(a_o_idx,to_o_idx) = prob;
                ++to_o_idx;
            }
            a_o_idx_map[performed_action][from_observation] = a_o_idx;
            ++a_o_idx;
        }
    }
}

GoalIteration::action_ptr_t GoalIteration::get_action(const_instance_ptr_t i) {
    // auto iterate
    if(auto_iterate) {
        iterate(pow(discount,observation_space->space_size()));
    }
    // get action (with randomization)
    observation_ptr_t o = i->observation;
    vector<action_ptr_t> optimal_actions;
    double max_value = -DBL_MAX;
    for(auto a : action_space) {
        double value = Q(a_o_idx_map[a][o]);
        if(value > max_value) {
            optimal_actions.clear();
            optimal_actions.push_back(a);
            max_value = value;
        } else if(value == max_value) {
            optimal_actions.push_back(a);
        }
    }
    return util::random_select(optimal_actions);
}

void GoalIteration::set_goal(observation_ptr_t o) {
    R = zeros(observation_space->space_size());
    bool found = false;
    int o_idx = 0;
    for(auto obs : observation_space) {
        if(o==obs) {
            found = true;
            R(o_idx) = 1;
            break;
        }
        ++o_idx;
    }
    // report error if goal observation was not found
    if(!found) {
        DEBUG_ERROR("Observation " << o << " is not within space of " << observation_space);
    }
}

void GoalIteration::print_matrices() const {
    Q.print("Q: ");
    R.print("R: ");
    V.print("V: ");
    p.print("p: ");
}

void GoalIteration::iterate(const double & threshold) {
    // update Q
    Q = p * (R + discount*V);
    // update V
    double max_diff = 0;
    int o_idx = 0;
    for(auto o : observation_space) {
        double old_value = V(o_idx);
        V(o_idx) = 0;
        for(auto a : action_space) {
            V(o_idx) = max(V(o_idx),Q(a_o_idx_map[a][o]));
        }
        max_diff = max(max_diff,fabs(old_value-V(o_idx)));
        ++o_idx;
    }
    // call recursively
    if(max_diff>threshold) {
        iterate(threshold);
    }
}

GoalIteration::color_vector_t GoalIteration::get_value_as_color() const {
    color_vector_t vec;
    int o_idx = 0;
    for(auto o : observation_space) {
        double v = V(o_idx)*(1-discount); // in [0,1]
        v = sqrt(v); // to make small value more visible
        vec.push_back(color_t(1-v,1,1-v));
        ++o_idx;
    }
    return vec;
}
