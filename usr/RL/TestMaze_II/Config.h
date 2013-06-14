#ifndef CONFIG_H_
#define CONFIG_H_

/* #include "Representation/Action.h" */
class Action;
class ActionIt;

/* #include "Representation/State.h" */
class State;
class StateIt;

#include "Representation/Reward.h"
/* class Reward; */
/* class RewardIt; */

/* #include "Representation/Instance.h" */
class Instance;
class InstanceIt;
class ConstInstanceIt;

#define USE_CONFIG_TYPEDEFS                                     \
    typedef Config::idx_t              idx_t;                   \
    typedef Config::size_t             size_t;                  \
    typedef Config::probability_t      probability_t;           \

#define USE_CONFIG_CONSTS                                               \
    static const Config::size_t   maze_x_size = Config::maze_x_size;    \
    static const Config::size_t   maze_y_size = Config::maze_y_size;    \
    static const Config::size_t   k           = Config::k;              \
    static const Config::reward_t max_reward  = Config::max_reward;

class Config {

public:

    //==========================//
    //  Typedefs and Constants  //
    //==========================//

    //----------------------//
    //   General Typedefs   //
    //----------------------//
    typedef long long int          idx_t;
    typedef unsigned long long int size_t;

    //----------------------//
    //   Maze Dimensions    //
    //----------------------//
    static const size_t maze_x_size;
    static const size_t maze_y_size;

    //-----------------//
    //   Max Reward    //
    //-----------------//
    static const reward_t max_reward;

    //----------------------//
    //   k-Markov Horizon   //
    //----------------------//
    static const size_t k;

    //-----------------//
    //   Probability   //
    //-----------------//
    typedef double probability_t;

};

#endif /* CONFIG_H_ */
