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

    //----------------------//
    //   k-Markov horizon   //
    //----------------------//

    static const unsigned long k;

    //-------------//
    //   actions   //
    //-------------//

    typedef unsigned long action_t;
    typedef unsigned long action_idx_t;
    enum ACTION {   STAY,      UP,    DOWN,    LEFT,   RIGHT, NUMBER_OF_ACTIONS };
    static const char* action_strings[5];
    static const unsigned long action_n;

    //-------------//
    //   states   //
    //-------------//

    typedef unsigned long state_t;
    typedef unsigned long state_idx_t;
    static const unsigned long maze_x_dim;
    static const unsigned long maze_y_dim;
    static const unsigned long state_n;

    //-------------//
    //   rewards   //
    //-------------//

    typedef double reward_t;
    typedef unsigned long reward_idx_t;
    static const reward_t      min_reward;
    static const reward_t      max_reward;
    static const reward_t      reward_increment;
    static const unsigned long reward_n;


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

    typedef std::vector<data_point_t> episode_t;
    typedef episode_t::iterator       episode_iterator_t;
    typedef episode_t::const_iterator const_episode_iterator_t;
    typedef const_episode_iterator_t  input_data_t;

    typedef unsigned long input_data_idx_t;
    typedef unsigned long output_data_idx_t;
    static const unsigned long input_n;
    static const unsigned long output_n;

    typedef std::vector<data_point_t> k_mdp_state_t;
    typedef unsigned long k_mdp_state_idx_t;
    static const unsigned long k_mdp_state_n;

    //=============//
    //  Functions  //
    //=============//

    static reward_idx_t      idx_from_reward(reward_t);
    static reward_t          reward_from_idx(reward_idx_t);

    static input_data_idx_t  idx_from_input(input_data_t);
    static episode_t         input_from_idx(input_data_idx_t);

    static output_data_idx_t idx_from_output(input_data_t);
    static output_data_idx_t idx_from_output(output_data_t);

    static k_mdp_state_idx_t idx_from_k_mdp_state(k_mdp_state_t);
    static k_mdp_state_t     k_mdp_state_from_idx(k_mdp_state_idx_t);

};

#endif /* DATA_H_ */
