#include "RandomSearch.h"

#include <util/util.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using util::random_select;

RandomSearch::RandomSearch(std::shared_ptr<AbstractEnvironment> environment,
                           double discount):
    AbstractSearchTree(environment,discount)
{
    init();
}

void RandomSearch::init() {
    environment->reset_state();
    best_action = random_select(environment->get_actions());
    best_return = std::numeric_limits<reward_t>::lowest();
    best_rollout.clear();
}

void RandomSearch::next() {
    environment->reset_state();
    action_handle_t potential_action = random_select(environment->get_actions());
    rollout_t potential_rollout;
    while(!environment->is_terminal_state()) {
        action_handle_t action;
        if(potential_rollout.empty()) {
            action = potential_action;
        } else {
            action = random_select(environment->get_actions());
        }
        RETURN_TUPLE(observation_handle_t, observation,
                     reward_t, reward) = environment->transition(action);
        potential_rollout.push_back(transition_t(action,observation,reward));
    }
    reward_t potential_return = 0;
    for(auto transition_it=potential_rollout.rbegin();
        transition_it!=potential_rollout.rend();
        ++transition_it) {
        RETURN_TUPLE(action_handle_t, action,
                     observation_handle_t, observation,
                     reward_t, reward) = *transition_it;
        potential_return *= discount;
        potential_return += reward;
    }
    if(potential_return>best_return) {
        best_return = potential_return;
        best_rollout = potential_rollout;
        best_action = potential_action;
    }
}

RandomSearch::action_handle_t RandomSearch::recommend_action() const{
    DEBUG_OUT(1,"Best action " << *best_action << " with return " << best_return);
    return best_action;
}

void RandomSearch::prune(const action_handle_t & action_taken,
                         const observation_handle_t & observation_made) {

    NAMED_RETURN_TUPLE(transition,
                       action_handle_t, action,
                       observation_handle_t, observation,
                       reward_t, reward);
    transition = best_rollout.front();
    if(*action!=*action_taken || *observation!=*observation_made) {
        DEBUG_OUT(1,"Unexpected transition --> reinitialize");
        init();
    } else {
        DEBUG_OUT(1,"Go on with expected trajectory");
        best_return = (best_return - reward)/discount;
        best_rollout.pop_front();
        transition = best_rollout.front();
        best_action = action;
    }
}

void RandomSearch::plot_graph(const char* file_name,
                              const char* command,
                              const char* parameters,
                              bool delete_dot_file) const {}
