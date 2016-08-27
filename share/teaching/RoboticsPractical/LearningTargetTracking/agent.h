#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <math.h>
#include "../interface/myBaxter.h"
#include "environment.h"
using namespace std;

class agent{
private:
    arr state;
    arr action;
    arr reward;
    arr summed_reward;
    arr f = ARR(0., 0.);
    arr f1 = ARR(0., 0.);
    arr w_x = ARR(0., 0.);
    arr w_y = ARR(0., 0.);
    arr theta_x = ARR(0., 0.);
    arr theta_y = ARR(0., 0.);
    arr delta;
    double gamma = 0.8;
    double alpha = 0.01;
    double beta = 0.01;
    arr state_space;

public:

    arr choose_action(arr f, arr theta_x, arr theta_y, double sigma, bool useRos);
    arr compute_td_error(arr reward, arr f1, arr w_x, arr w_y, arr f);
    void critic_update(arr &w_x, arr &w_y, arr delta, arr f);
    void actor_update(arr &theta_x, arr &theta_y, arr f, arr w_x, arr w_y);
    void learn(environment &env, bool useRos);
    void test(environment &env, bool useRos);
    void writeParameter(bool useRos);
    void readParameter(bool fromRos);
};

#endif // AGENT_H
