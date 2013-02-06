#ifndef DATA_H_
#define DATA_H_

#include <vector>
#include <tuple>
#include <math.h>

class Data {

public:

    //==========================//
    //  Typedefs and Constants  //
    //==========================//

    //-------------//
    //   general   //
    //-------------//
    typedef long long idx_t;
    typedef unsigned long long size_t;

    //----------------------//
    //   k-Markov horizon   //
    //----------------------//

    static const size_t k;

    //-------------//
    //   actions   //
    //-------------//

    typedef idx_t action_t;
    typedef idx_t action_idx_t;
    enum ACTION { NULL_ACTION = -1, STAY, UP, DOWN, LEFT, RIGHT, NUMBER_OF_ACTIONS };
    static const char* action_strings[5];
    static const size_t action_n;

    //-------------//
    //   states   //
    //-------------//

    typedef idx_t state_t;
    typedef idx_t state_idx_t;
    static const size_t maze_x_dim;
    static const size_t maze_y_dim;
    static const size_t state_n;

    //-------------//
    //   rewards   //
    //-------------//

    typedef double reward_t;
    typedef idx_t reward_idx_t;
    static const reward_t min_reward;
    static const reward_t max_reward;
    static const reward_t reward_increment;
    static const size_t reward_n;

    typedef double value_t;


    //------------------//
    //   probability   //
    //------------------//

    typedef double probability_t;

    //------------------//
    //   input/output   //
    //------------------//

    struct data_point_t {
        data_point_t(action_t a = action_t(), state_t  s = state_t(), reward_t r = min_reward):
            action(a), state(s), reward(r) {}
        bool operator==(const data_point_t& other) const { return (action==other.action && state==other.state && reward==other.reward); }
        bool operator!=(const data_point_t& other) const { return !(*this==other); }
        bool operator<(const data_point_t& other) const {
            if(action<other.action) return true;
            else if(action>other.action) return false;
            else if(state<other.state) return true;
            else if(state>other.state) return false;
            else if(reward<other.reward) return true;
            else if(reward>other.reward) return false;
            else return false;
        }
        action_t action;
        state_t  state;
        reward_t reward;
    };

    struct output_data_t {
        output_data_t(state_t  s, reward_t r): state(s), reward(r) {}
        state_t  state;
        reward_t reward;
    };

    class OutputIterator {
    public:
        OutputIterator();
        ~OutputIterator();
        OutputIterator& operator++();
        output_data_t operator*() const;
        bool end() const;
    private:
        state_t current_state;
        reward_t current_reward;
    };

    typedef std::vector<data_point_t> episode_t; ///< [0] is present [-k] is k time steps in the past
    typedef episode_t::iterator       episode_iterator_t;
    typedef episode_t::const_iterator const_episode_iterator_t;
    typedef const_episode_iterator_t  input_data_t;

    typedef idx_t input_data_idx_t;
    typedef idx_t output_data_idx_t;
    static const size_t input_n;
    static const size_t output_n;

    typedef std::vector<data_point_t> k_mdp_state_t; ///< [0] is present [k] is k time steps in the past
    typedef idx_t                     k_mdp_state_idx_t;
    static const size_t k_mdp_state_n;

    //=============//
    //  Functions  //
    //=============//

    static action_idx_t      idx_from_action(action_t);
    static action_t          action_from_idx(action_idx_t);

    static state_idx_t       idx_from_state(state_t);
    static state_t           state_from_idx(state_idx_t);

    static reward_idx_t      idx_from_reward(reward_t);
    static reward_t          reward_from_idx(reward_idx_t);

    static input_data_idx_t  idx_from_input(input_data_t);
    static episode_t         input_from_idx(input_data_idx_t);

    static output_data_idx_t idx_from_output(input_data_t);
    static output_data_idx_t idx_from_output(output_data_t);

    static k_mdp_state_idx_t idx_from_k_mdp_state(k_mdp_state_t);
    static k_mdp_state_t     k_mdp_state_from_idx(k_mdp_state_idx_t);

    static idx_t             prediction_idx(k_mdp_state_t,action_t,state_t,reward_t);
    static idx_t             state_action_idx(k_mdp_state_t,action_t);

};

#endif /* DATA_H_ */
