#ifndef MDP_FUNCTIONAL_H
#define MDP_FUNCTIONAL_H




#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>
#include "plotUtil.h"
#include "traj_factory.h"
#include "task_door.h"
#include <Ors/ors.h>



namespace mdp{





class RKHSPol{

private:
    //const ENVIRONMENT& Env;
    DoorTask *task;// = new DoorTask(world);
    arr Xdemo;
    arr FLdemo, Mdemo;
    arr paramLim;


    int NumEps; //episodes for each iteration of gradient evalation
    int Horizon;
    int NumIterations; //total number of gradient updates.

    double RBFVariance;


    //functional policy: two dimensional array: store and 0) alpha_i (dim_A) 1-) centres (s_i)(dim_S),
    //parametric policy:dim 0) w_i ; dim 1-)centre c_i vector

    arr FuncPolicy;
    uint currIteration;
    uint NumCentre; //Sparsification
    arr Sigma;



    /*/ 0) RBF;
        2) polynomial ((xy +c)^d; d=5; c=10.), this makes plain PG converge slowly
        1) polynomial ((xy +c)^d; d=5; c=1.) , this makes plain PG converge normally
    /*/

    uint Kernel_Type;

    double TrajectoryKernel(const arr& states1, const arr& actions1,const arr& states2, const arr& actions2);
    void GramMatrix(const arr&  DATA_S,const arr&  DATA_A, arr& Gram);   

    double lineSearch(const arr& gradient);

    bool forwardDynamics(arr action, arr state, arr &nxtState, double &reward);



public:
    arr StartingState;
    uint dim_A;
    uint dim_S;

    double ACTION_MAX;
    double ACTION_MIN;


    RKHSPol(ors::KinematicWorld world, arr Xdemo,arr FLdemo, arr Mdemo, arr paramLim, uint numCentre, uint horizon,uint numEps,uint kernel_Type, int numIterations);

    void setStart(const arr &start);

    //return cummulative after one run.
    arr run();
    double rollout(const arr ht, arr& states, arr& actions, arr& rewards);    
    void evaluate(const arr state, const arr &ht, arr &hs);
    double kernelFunc(const arr state1, const arr state2);
    void sampleAction(const arr ht, const arr state, arr& action);

    void update(arr &newPol, arr& states, arr& actions, double scale, const arr oldPol);


    arr runNPG(); //natural functional policy gradient
    arr runPG(); //plain functional policy gradient

    uint Algorithm;

    void loadOldFuncPolicy();




    ~RKHSPol(){
        delete task;
        task = NULL;
    }


};









}


#endif // MDP_FUNCTIONAL_H
