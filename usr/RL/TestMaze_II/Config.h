#ifndef CONFIG_H_
#define CONFIG_H_

#define USE_CONFIG_TYPEDEFS                                     \
    typedef Config::idx_t              idx_t;                   \
    typedef Config::size_t             size_t;                  \
    typedef Config::probability_t      probability_t;           \

#define USE_CONFIG_CONSTS                                               \
    static const Config::size_t   maze_x_size = Config::maze_x_size;    \
    static const Config::size_t   maze_y_size = Config::maze_y_size;    \
    static const Config::size_t   k           = Config::k;

class Reward;

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
    static const Reward min_reward;
    static const Reward max_reward;
    static const Reward reward_increment;

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
