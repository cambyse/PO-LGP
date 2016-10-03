#include <Core/util.h>
#include <Core/array.h>
#include <stdlib.h>

#include "RL/cartPole_Env.h"
#include "RL/cartPole_Pol.h"
#include "RL/cartPole_Filter.h"
#include "RL/Optimization.h"
#include "RL/PolicySearch.h"
#include "RL/Auxiliary.h"


using namespace std;
using namespace mdp; //Environment + Policy + Filter + Policy Search + Optimization
using namespace mlr;


int main(int argc, char *argv[]){

    uint horizon = 5000;
    uint numEps = 200;
    uint numIter = 200;
    double discount = 1.;

    uint stateDim = 4;
    uint actionDim = 1;
    uint policyDim = 4; //4 for observable, 10 for PO
    arr startState(stateDim); startState.setZero();
    arr startFeature(policyDim); startFeature.setZero();
    uint control = 0; // 0 = Bag Bag control, 1 = stochastic control
    //Observ = 0 => POMDP, by default = 1
    uint observ = 1; //Careful! Also change dimension of policy

    cartPole_Env env(startState, control, observ);
    cartPole_Pol pol(actionDim, policyDim);
    cartPole_Filter filter(startFeature);
    Optimization solver;

    PolicySearch cartPole(env, pol, filter, solver, horizon, numEps, numIter, discount);

    arr x = ARR(0.5, 0.1, -0.3, -0.4);
    myCheckGradient(cartPole, x);

//    mlr::rnd.clockSeed();
//    cartPole.runREINFORCE();
//    cartPole.runGPOMDP();

//    arr x = ARR(2, 1, 3);
//    arr y = ARR(1, 0, 2);
//    arr z = x%y;
//    cout<<z;

    return 0;
}

