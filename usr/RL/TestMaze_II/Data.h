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
    static const double min_reward;
    static const double max_reward;
    static const double reward_increment;
    static const int    reward_n;

    //---other---//
    typedef double                                probability_t;
    typedef std::tuple<action_t,state_t,reward_t> data_point_t;
    typedef std::vector<data_point_t>             episode_t;
    typedef episode_t::const_iterator             const_episode_iterator_t;
    typedef const_episode_iterator_t              input_data_t;
    typedef const data_point_t                    output_data_t;
    static const int output_n;

    //-------------//
    //  Functions  //
    //-------------//

    static int output_idx(input_data_t data);
    static int output_idx(input_data_t, output_data_t data_predict);

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
        action_t current_action;
        state_t current_state;
        reward_t current_reward;
    };

};

#endif /* DATA_H_ */
