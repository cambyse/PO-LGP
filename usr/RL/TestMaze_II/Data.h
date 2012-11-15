#ifndef DATA_H_
#define DATA_H_

#include <vector>
#include <tuple>


class Data {

public:

    //---actions---//
    enum ACTION {   STAY,      UP,    DOWN,    LEFT,   RIGHT, NUMBER_OF_ACTIONS };
    static const char* action_strings[5];
    typedef int action_t;
    static const int action_n;

    //---states---//
    static const int maze_x_dim;
    static const int maze_y_dim;
    typedef int state_t;
    static const int state_n;

    //---rewards---//
    typedef double reward_t;
    static const double min_reward;
    static const double max_reward;
    static const double reward_increment;
    static const int    reward_n;

    //---other---//
    typedef double                                probability_t;
    typedef std::tuple<action_t,state_t,reward_t> data_point_t;
    typedef std::vector<data_point_t>             episode_t;
    typedef episode_t::const_iterator             const_episode_iterator_t;
    typedef const_episode_iterator_t input_data_t;
    typedef const data_point_t       output_data_t;

};

#endif /* DATA_H_ */
