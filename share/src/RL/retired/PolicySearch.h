
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_PolicySearch_H
#define MDP_PolicySearch_H


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>

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
    uint getNumEps();
    Policy* getPolicy();

    //Specific functions
    double rollout(const arr &theta);
    double just_rollout(arr& features, arr& actions, arr& rewards, const arr& theta, uint numSteps);
    double grad_rollout(arr& gradLogRet, arr& actions, arr& rewards, const arr &theta, uint numSteps);

    //for implemented algorithms
    double updateREINFORCE(arr& gradJ, const arr& theta);
    arr runREINFORCE();

    double updateGPOMDP(arr& gradJ, const arr& theta);
    arr runGPOMDP();

    //Modified functions
    double updateREINFORCE_ver1(arr& gradJ, const arr& theta);
    double updateGPOMDP_ver1(arr &gradJ, const arr &theta);
};


}

#endif
