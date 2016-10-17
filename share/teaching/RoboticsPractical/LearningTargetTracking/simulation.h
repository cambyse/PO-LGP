#ifndef SIMULATION_H
#define SIMULATION_H


#include <Core/util.h>
#include "../interface/myBaxter.h"
#include <Control/taskController.h>
#include <vector>
#include <Core/array.h>
#include <fstream>


using namespace std;


class simulation
{
private:
    const int precision_length = 20;
    const int precision_width = 20;
    const double gamma = 0.7;
    const double lambda = 0.8;
    const double epsilon = 0.1;
    const double alpha = 0.15;
    const int number_of_episodes = 500;
    const int number_of_steps = 100;
    const int n_actions = 4;

    const int position_space[2] = {precision_width, precision_length};
    const int state_space[2] = {2*position_space[0]-1, 2*position_space[1]-1};


    intA initial_state;
    intA state;
    intA new_state;

    intA initial_position_robot;
    intA initial_position_marker;
    intA position_robot;
    intA position_marker;

    double reward;
    int action;
    int next_action;
    int greedy_action;

    arr Q;
    arr e;

public:


    simulation();
    int e_greedy(intA state_now, double eps);
    void state_transition();
    double get_reward();
    void random_marker_walk();
    void write_Q_values();
    void run();

};

#endif // SIMULATION_H
