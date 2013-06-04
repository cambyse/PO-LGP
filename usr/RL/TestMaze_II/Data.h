#ifndef DATA_H_
#define DATA_H_

#define USE_DATA_TYPEDEFS                                       \
    typedef Data::idx_t         idx_t;                          \
    typedef Data::size_t        size_t;                         \
    typedef Data::probability_t probability_t;
#define USE_DATA_CONSTS                                         \
    using Data::maze_x_size;                                    \
    using Data::maze_y_size;                                    \
    using Data::k;


class Data {

public:

    //==========================//
    //  Typedefs and Constants  //
    //==========================//

    //-------------//
    //   General   //
    //-------------//
    typedef long long int idx_t;
    typedef unsigned long long int size_t;

    //----------------------//
    //   Maze Dimensions    //
    //----------------------//
    static const size_t maze_x_size;
    static const size_t maze_y_size;

    //----------------------//
    //   k-Markov Horizon   //
    //----------------------//
    static const size_t k;

    //------------------//
    //   Probability   //
    //------------------//

    typedef double probability_t;

};

#endif /* DATA_H_ */
