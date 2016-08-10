#include<Core/util.h>
#include<Core/array.h>
#include <stdlib.h>

#include "RL/cartPole_Env.h"
#include "RL/cartPole_Pol.h"
#include "RL/cartPole_Filter.h"
#include "RL/Optimization.h"
#include "RL/PolicySearch.h"


using namespace std;
using namespace mdp; //Environment + Policy + Filter + Policy Search + Optimization
using namespace mlr;


int main(int argc, char *argv[]){

    //Environment* env;
    Policy* pol;
    Filter* filter;
//    Optimization* solver;

    uint horizon = 5000;
    uint numEps = 200;
    uint numIter = 200;
    double discount = 1.;

    uint stateDim = 4;
    uint actionDim = 1;
    arr startState(stateDim);
    startState.setZero();

    cartPole_Env env(startState);
    pol = new cartPole_Pol(actionDim, stateDim);
    filter = new cartPole_Filter();
    Optimization solver(false);

    PolicySearch cartPole(env, *pol, *filter, solver, horizon, numEps, numIter, discount);

    mlr::rnd.clockSeed();
    cartPole.updateREINFORCE();
//    cartPole.updateGPOMDP();

//    arr x = ARR(2., 3., 1.);
//    arr y = ARR(1., 2., 3.);
//    arr z;
//    indexWiseProduct(z,x,y);
//    cout<<z<<endl;

  return 0;
}

