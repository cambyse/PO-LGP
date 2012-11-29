#ifndef DATA_H_
#define DATA_H_

#include <vector>
#include <tuple>
#include <math.h>

class Data {

public:

    //--------------------------//
    //  Typedefs and Constants  //
    //--------------------------//

    //------k-Markov horizon----//
    static const int k;

    //---actions---//
    typedef int action_t;
    enum ACTION {   STAY,      UP,    DOWN,    LEFT,   RIGHT, NUMBER_OF_ACTIONS };
    static const char* action_strings[5];
    static const int action_n;

    //---states---//
    typedef int state_t;
    static const int maze_x_dim;
    static const int maze_y_dim;
    static const int state_n;

    //---rewards---//
    typedef double reward_t;
    static const reward_t min_reward;
    static const reward_t max_reward;
    static const reward_t reward_increment;
    static const int      reward_n;


    //---probability---//
    typedef double probability_t;

    //---input/output---//
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
    typedef std::vector<data_point_t> episode_t;
    typedef episode_t::iterator       episode_iterator_t;
    typedef episode_t::const_iterator const_episode_iterator_t;
    typedef const_episode_iterator_t  input_data_t;
    static const int input_n;
    static const int output_n;

    //-------------//
    //  Functions  //
    //-------------//

    static unsigned long idx_from_reward(reward_t);
    static reward_t reward_from_idx(unsigned long);
    static unsigned long idx_from_input(input_data_t);
    static episode_t input_from_idx(unsigned long);
    static unsigned long idx_from_output(input_data_t);
    static unsigned long idx_from_output(output_data_t);

    //-----------//
    //  Classes  //
    //-----------//
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

};

#endif /* DATA_H_ */
