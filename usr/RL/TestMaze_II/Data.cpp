#include "Data.h"

#include <math.h>

const char*  Data::action_strings[5] = { " STAY", "   UP", " DOWN", " LEFT", "RIGHT" };
const int    Data::action_n          = NUMBER_OF_ACTIONS;
const int    Data::maze_x_dim        = 2;
const int    Data::maze_y_dim        = 2;
const int    Data::state_n           = maze_x_dim*maze_y_dim;
const double Data::min_reward        = 0.0;
const double Data::max_reward        = 1.0;
const double Data::reward_increment  = 1.0;
const int    Data::reward_n          = floor((max_reward-min_reward)/reward_increment)+1;
