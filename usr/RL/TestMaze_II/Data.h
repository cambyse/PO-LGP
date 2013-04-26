#ifndef DATA_H_
#define DATA_H_

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
    static const size_t maze_x_size = 2;
    static const size_t maze_y_size = 2;

    //----------------------//
    //   k-Markov Horizon   //
    //----------------------//
    static const size_t k = 6;

    //------------------//
    //   Probability   //
    //------------------//

    typedef double probability_t;

};

#endif /* DATA_H_ */
