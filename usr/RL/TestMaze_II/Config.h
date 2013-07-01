#ifndef CONFIG_H_
#define CONFIG_H_

//#define BATCH_MODE
//#define BATCH_MODE_QUIET

#define USE_CONFIG_TYPEDEFS                                     \
    typedef Config::idx_t              idx_t;                   \
    typedef Config::size_t             size_t;                  \
    typedef Config::probability_t      probability_t;           \

#define USE_CONFIG_CONSTS                                               \
    static const Config::size_t   maze_x_size = Config::maze_x_size;    \
    static const Config::size_t   maze_y_size = Config::maze_y_size;    \
    static const Config::size_t   k           = Config::k;              \
    static const Config::reward_t min_reward  = Config::min_reward;     \
    static const Config::reward_t max_reward  = Config::max_reward;

/* #include "Representation/SequentialReward.h" */
#include "Representation/EnumeratedReward.h"

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
    /* typedef SequentialReward reward_t; */
    /* typedef SequentialRewardIt rewardIt_t; */
    typedef EnumeratedReward reward_t;
    typedef EnumeratedRewardIt rewardIt_t;
    static const reward_t min_reward;
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
