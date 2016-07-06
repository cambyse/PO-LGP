
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_FUNCTIONAL_H
#define MDP_FUNCTIONAL_H




#include <iostream>
#include <stdint.h>
#include <cstring>

#include<Core/util.h>
#include<Core/array.h>
#include "environment.h"



namespace mdp{



class RKHSPol{

private:



    const ENVIRONMENT& Env;
    int NumEps; //episodes for each iteration of gradient evalation
    int Horizon;
    int NumIterations; //total number of gradient updates.

    arr RBFVariance2;

    //functional policy: two dimensional array: store and 0) alpha_i (dim_A) 1-) centres (s_i)(dim_S),
    //parametric policy:dim 0) w_i ; dim 1-)centre c_i vector

    arr FuncPolicy;
    arr FuncVariances; // variance functional
    uint NumCentre;    //Sparsification
    arr SigmaInv;
    arr KernelVars;



    /*/ 0) RBF;
        2) polynomial ((xy +c)^d; d=5; c=10.), this makes plain PG converge slowly
        1) polynomial ((xy +c)^d; d=5; c=1.) , this makes plain PG converge normally
    /*/

    uint Kernel_Type;

    double TrajectoryKernel(const arr& states1, const arr& actions1,const arr& states2, const arr& actions2);
    void GramMatrix(const arr&  DATA_S,const arr&  DATA_A, arr& Gram);

    void Sparsification(arr& Dictionary, const arr base);
    void Regression(arr& Dictionary, arr&Kernel_S, const arr STATES, const arr ACTIONS, const arr REWARDS);
    void KernelMatchingPursuit(arr& Dictionary, const arr Dataset, arr Residue, const arr Kernel);

    double lineSearch(const arr& gradient);

    void EvalPolicy(const arr funcPolicy, double& Total_R);



public:

    uint dim_A;
    uint dim_S;

    RKHSPol(const ENVIRONMENT& env, int numCentre, uint horizon, uint numEps, int numIterations);

    //return cummulative after one run.
    arr run();

    double rollout(const arr ht, arr& states, arr& actions, arr& rewards);

    void evaluate(const arr state, const arr &ht, arr &hs);
    double kernelFunc(const arr state1, const arr state2);
    void sampleAction(const arr ht, const arr state, arr& action);
    void update(arr &newPol, arr& states, arr& actions, double scale, const arr oldPol);


    //for implemented algorithms
    arr runRKHS();


    ~RKHSPol(){}


};









}


#endif // MDP_FUNCTIONAL_H
