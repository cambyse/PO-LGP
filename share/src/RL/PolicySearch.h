
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_PolicySearch_H
#define MDP_PolicySearch_H


#include <iostream>
#include <stdint.h>
#include <cstring>

#include<Core/util.h>
#include<Core/array.h>

#include "Environment.h"
#include "Policy.h"
#include "Filter.h"
#include "Optimization.h"


namespace mdp {


class PolicySearch{

protected:
    Environment& Env;
    Policy& Pol;
    Filter& Fil;
    Optimization& Solver;
    uint Horizon;
    uint NumEps;
    uint NumIter;
    double Discount;
    arr step;

public:
    //Constructor and destructor
    PolicySearch() = default;
    PolicySearch(Environment& env, Policy& pol, Filter& fil, Optimization& solver, uint horizon, uint numEps, uint numIter, double discount);
    ~PolicySearch();

    //Inner access
    uint getHorizon();

    //Specific functions
    double rollout(const arr& theta, arr& observations, arr& actions, arr& rewards, uint numSteps);

    //for implemented algorithms
    double REINFORCE(arr& theta, arr& gradJ);
    arr updateREINFORCE();

    double GPOMDP(arr& theta, arr& gradJ);
    arr updateGPOMDP();

};


}

#endif
