/** \file UCT_Example.cpp
 *
 * \example UCT_Example.cpp This is an example that implements a very primitive
 * type of UCT based on the AbstractEnvironment interface. */

#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <tuple>
#include <memory>
#include <limits>
#include <math.h>

// change this to whatever implementation of the AbstractEnvironment interface
#include "../Environment/MC_versus_DP.h"

using std::shared_ptr;
using std::cout;
using std::endl;
using std::tuple;

// basic types
typedef AbstractEnvironment::action_handle_t action_t;           // shared_ptr
typedef AbstractEnvironment::observation_handle_t observation_t; // shared_ptr
typedef AbstractEnvironment::reward_t reward_t;                  // double

// sets and maps
typedef std::unordered_set<action_t,
                           AbstractEnvironment::ActionHash,
                           AbstractEnvironment::ActionEq> action_set_t;
typedef std::unordered_set<observation_t,
                           AbstractEnvironment::ObservationHash,
                           AbstractEnvironment::ObservationEq> observation_set_t;
template<class T>
using action_map_t = std::unordered_map<action_t, T,
                                        AbstractEnvironment::ActionHash,
                                        AbstractEnvironment::ActionEq>;
template<class T>
using observation_map_t = std::unordered_map<observation_t, T,
                                             AbstractEnvironment::ObservationHash,
                                             AbstractEnvironment::ObservationEq>;

// "nodes" of the search tree
struct Node {
    Node(shared_ptr<Node> backlink): backlink(backlink) {}
    int count_sum = 0;
    action_map_t<double> return_sums;
    action_map_t<int> action_counts;
    action_map_t<observation_map_t<shared_ptr<Node>>> nodes;
    std::weak_ptr<Node> backlink;
    double last_reward;
    action_t action;
    observation_t observation;
};

int main(int argn, char ** args) {

    // random seeds
    srand(time(nullptr));
    srand48(time(nullptr));

    // using AbstractEnvironment interface
    std::shared_ptr<AbstractEnvironment> env(new MC_versus_DP());

    // make sure we can to standard rollouts to terminal state
    assert(env->has_terminal_state());

    // some variables that will be used throughout all loops
    action_t action;
    observation_t observation;
    reward_t reward;
    std::tuple<observation_t&,reward_t&> observation_and_reward (observation,reward);
    shared_ptr<Node> root_node(new Node(nullptr));

    // some parameters for UCT and output
    int rollout_n = 100;
    double Cp = 2;
    double discount = 1;
    int verbosity = 1;

    // make planned actions until reaching terminal state
    int action_count = 0;
    while(!env->is_terminal_state()) {
        ++action_count;
        if(verbosity>0) cout << "Action " << action_count << endl;
        // planning
        for(int rollout_count=1; rollout_count<=rollout_n; ++rollout_count) {
            auto current_node = root_node;
            // do a rollout
            if(verbosity>1) cout << "    Rollout " << rollout_count << endl;
            env->reset_state();
            while(!env->is_terminal_state()) {
                // choose action using UCB1 without (!) random tie breaking
                action = nullptr;
                double max_bound = std::numeric_limits<double>::lowest();
                for(auto a : env->get_actions()) {
                    int counts = current_node->action_counts[a];
                    if(counts==0) {
                        action = a;
                        break;
                    }
                    double bound = current_node->return_sums[a]/counts;        // mean return
                    bound += 2*Cp*sqrt(2*log(current_node->count_sum)/counts); // exploration term
                    if(bound>max_bound) {
                        action = a;
                        max_bound = bound;
                    }
                }
                assert(action!=nullptr);
                // perform transition
                observation_and_reward =  observation_and_reward = env->transition(action);
                // update counters and current node
                ++(current_node->count_sum);
                ++(current_node->action_counts[action]);
                current_node->last_reward = reward;
                if(current_node->nodes[action][observation]==nullptr) current_node->nodes[action][observation].reset(new Node(current_node));
                current_node = current_node->nodes[action][observation];
                current_node->action = action;
                current_node->observation = observation;
                if(verbosity>2) cout << "        transition: " << *action << " --> " << *observation << ", " << reward << endl;
            }
            // backup return
            if(verbosity>2) cout << "        backup" << endl;
            double discounted_return = 0;
            while(!current_node->backlink.expired()) {
                action = current_node->action;
                observation = current_node->observation;
                current_node = current_node->backlink.lock();
                discounted_return = current_node->last_reward + discount*discounted_return;
                current_node->return_sums[action] += discounted_return;
            }
        }
        env->reset_state();
        // find best action
        action = nullptr;
        double max_return = std::numeric_limits<double>::lowest();
        for(auto a : env->get_actions()) {
            double discounted_return = root_node->return_sums[a]/root_node->count_sum;
            if(discounted_return>max_return) {
                action = a;
                max_return = discounted_return;
            }
        }
        assert(action!=nullptr);
        // perform transition
        observation_and_reward = env->transition(action);
        env->make_current_state_new_start();
        // go into branch (this also deletes all other branches)
        root_node = root_node->nodes[action][observation];
        // print transition
        if(verbosity>0) cout << "    performed transition " << *action << " --> " << *observation << ", " << reward << endl;
    }

    return 0;
}
