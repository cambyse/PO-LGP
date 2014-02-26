#include "LookAheadPolicy.h"

#include "../debug.h"

LookAheadPolicy::LookAheadPolicy(const double& discount,
                                 std::shared_ptr<const Predictor> pred,
                                 const bool& prune,
                                 const int& max_size
    ):
    look_ahead_search(discount), predictor(pred),
    prune_tree(prune), valid_tree_exists(false),
    last_action(action_ptr_t()),
    max_tree_size(max_size)
{
    action_ptr_t action_space;
    observation_ptr_t observation_space;
    reward_ptr_t reward_space;
    predictor->get_spaces(action_space, observation_space, reward_space);
    look_ahead_search.set_spaces(action_space, observation_space, reward_space);
}

LookAheadPolicy::action_ptr_t LookAheadPolicy::get_action(const instance_t * instance) {

    // debug output
    if(DEBUG_LEVEL>2) {
        DEBUG_OUT(0,"Before planning:");
        look_ahead_search.print_tree_statistics();
    }

    // build/expand tree
    action_ptr_t action;
    if(prune_tree && valid_tree_exists && last_action!=action_ptr_t()) {
        // prune tree
        look_ahead_search.prune_tree(last_action, instance, *predictor);
        // debug output
        if(DEBUG_LEVEL>2) {
            DEBUG_OUT(0,"After pruning:");
            look_ahead_search.print_tree_statistics();
        }
        // expand tree
        look_ahead_search.fully_expand_tree(*predictor, max_tree_size);
    } else {
        // clear tree
        look_ahead_search.clear_tree();
        // build tree
        look_ahead_search.build_tree(instance, *predictor, max_tree_size);
    }

    // debug output
    if(DEBUG_LEVEL>2) {
        DEBUG_OUT(0,"After planning:");
        look_ahead_search.print_tree_statistics();
    }

    // get action
    action = look_ahead_search.get_optimal_action();

    // remember values
    valid_tree_exists = true;
    last_action = action;

    return action;
}
