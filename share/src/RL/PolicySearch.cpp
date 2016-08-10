
#include<Core/util.h>
#include<Core/array.h>

#include <assert.h>
#include <float.h>

#include "PolicySearch.h"


/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


using namespace mlr;

namespace mdp {


PolicySearch::PolicySearch(Environment& env, Policy& pol, Filter& fil, Optimization& solver, uint horizon, uint numEps, uint numIter, double discount)
    : Env(env),
      Pol(pol),
      Fil(fil),
      Solver(solver),
      Horizon(horizon),
      NumEps(numEps),
      NumIter(numIter),
      Discount(discount)
{
    assert(Discount >= 0 && Discount <= 1);
    step = ~ARR(0.7, 0.7, 0.7, 0.7);
}

PolicySearch::~PolicySearch() {}

uint PolicySearch::getHorizon()
{
    return Horizon;
}


double PolicySearch::rollout(const arr& theta, arr& observations, arr& actions, arr& rewards, uint numSteps)
{
     observations.clear();
     actions.clear();
     rewards.clear();

     Env.resetState();
     Fil.clearHistory();
     arr currentAgentObs; currentAgentObs.setZero();
     arr action;
     arr perception;

     double reward;
     double totalReturn = 0.0;  

     bool terminal;

     for(uint t=0; t<numSteps; t++)
     {
        Pol.sampleAction(currentAgentObs, theta, action);
        observations.append(~currentAgentObs);

        //Append as a line - in actions, a row is an action
        actions.append(~action);

        terminal = Env.transition(action, perception, reward);
        Fil.savePerception(perception);
        rewards.append(reward);

        Fil.computeEstimate();
        currentAgentObs = Fil.getObsEstimate();

        totalReturn += pow(Discount,t) * reward;

        if(terminal) break;
     }

     return totalReturn;
}


double PolicySearch::REINFORCE(arr& theta, arr& gradJ)
{
    arr observations, actions, rewards;
    arr gradLog, nom, den, b;
    nom.resizeAs(gradJ);
    den.resizeAs(gradJ);
    b.resizeAs(gradJ);
    double Reward, total, average;
    total = .0;

    for(uint ep=0; ep<NumEps; ep++)
    {
        nom.setZero(); den.setZero();

        for(uint epBase=0; epBase<NumEps; epBase++)
        {
            Reward = rollout(theta, observations, actions, rewards, Horizon);
            Pol.gradLogPol(observations, theta, actions, gradLog);
            //How to do element wise multiplication?
            for(uint i=0; i<gradLog.d0; i++)
            {
                for(uint index=0; index<gradLog.d1; index++)
                {
                    nom(i,index) += gradLog(i,index) * gradLog(i,index) * Reward;
                    den(i,index) += gradLog(i,index) * gradLog(i,index);
                }
            }
        }

        nom /= (double)NumEps;
        den /= (double)NumEps;
        for(uint i=0; i<nom.d0; i++)
            for(uint index=0; index<nom.d1; index++)
                b(i,index) = nom(i,index) / den(i,index);

        Reward = rollout(theta, observations, actions, rewards, Horizon);
        total += Reward;

        Pol.gradLogPol(observations, theta, actions, gradLog);
        for(uint i=0; i<gradLog.d0; i++)
            for(uint index=0; index<gradLog.d1; index++)
                gradJ(i,index) += gradLog(i,index) * (Reward - b(i,index));
    }

    gradJ /= (double)NumEps;
    average = (double) total/NumEps;
    return average;
}


arr PolicySearch::updateREINFORCE()
{
    arr theta; theta.resize(Pol.getActionDim()*Pol.getPolicyDim()); theta.setZero();
    arr gradJ; gradJ.resize(Pol.getActionDim(), Pol.getPolicyDim());
    arr gradJOld; gradJOld.resizeAs(gradJ); gradJOld.setZero();
    arr rewardIter(NumIter);
    double rewardMean;

    for(uint iter=0; iter<NumIter; iter++)
    {
        gradJ.setZero();
        rewardMean = REINFORCE(theta, gradJ);
        Solver.RPROP(step, gradJ, gradJOld, theta);
//        cout<<step<<endl<<gradJOld<<endl;
        cout<<" The expected reward of iteration "<< iter <<" is: "<< rewardMean <<endl;
        rewardIter(iter) = rewardMean;
    }

    return rewardIter;
}


double PolicySearch::GPOMDP(arr& theta, arr& gradJ)
{
    arr observations, actions, rewards;
    arr obsMain, actMain, rewMain;
    arr auxObs, auxAct;
    arr gradLog, gradLogMain;
    arr nom, den, b;
    nom.resizeAs(gradJ);
    den.resizeAs(gradJ);
    b.resizeAs(gradJ);
    double Reward, total, average;
    total = .0;

    for(uint ep=0; ep<NumEps; ep++)
    {
//        mlr::rnd.clockSeed();
        Reward = rollout(theta, observations, actions, rewards, Horizon);
        total += Reward;
        obsMain = observations;
        actMain = actions;
        rewMain = rewards;

        for(uint j=0; j<observations.d0; j++)
        {
            auxObs = obsMain.sub(0,j, 0,-1);
            auxAct = actMain.sub(0,j, 0,-1);
            Pol.gradLogPol(auxObs, theta, auxAct, gradLog);
            gradLogMain = gradLog;

            nom.setZero(); den.setZero();
            for(uint epBase=0; epBase<NumEps; epBase++)
            {
                Reward = rollout(theta, observations, actions, rewards, j);
                Pol.gradLogPol(observations, theta, actions, gradLog);

                //How to do element wise multiplication?
                for(uint i=0; i<gradLog.d0; i++)
                {
                    for(uint index=0; index<gradLog.d1; index++)
                    {
                        nom(i,index) += gradLog(i,index) * gradLog(i,index) * rewMain(j);
                        den(i,index) += gradLog(i,index) * gradLog(i,index);
                    }
                }

                nom /= (double)NumEps;
                den /= (double)NumEps;
                for(uint i=0; i<nom.d0; i++)
                    for(uint index=0; index<nom.d1; index++)
                        b(i,index) = nom(i,index) / den(i,index);
            }

            for(uint i=0; i<gradLogMain.d0; i++)
                for(uint index=0; index<gradLogMain.d1; index++)
                    gradJ(i,index) += gradLogMain(i,index) * (rewMain(j) - b(i,index));
        }
    }

    gradJ /= (double)NumEps;
    average = (double) total/NumEps;
    return average;
}


arr PolicySearch::updateGPOMDP()
{
    arr theta; theta.resize(Pol.getActionDim()*Pol.getPolicyDim()); theta.setZero();
    arr gradJ; gradJ.resize(Pol.getActionDim(), Pol.getPolicyDim());
    arr gradJOld; gradJOld.resizeAs(gradJ); gradJOld.setZero();
    arr rewardIter(NumIter);
    double rewardMean;

    for(uint iter=0; iter<NumIter; iter++)
    {
        gradJ.setZero();
        rewardMean = GPOMDP(theta, gradJ);
        Solver.RPROP(step, gradJ, gradJOld, theta);
        cout<<" The expected reward of iteration "<< iter <<" is: "<< rewardMean <<endl;
        rewardIter(iter) = rewardMean;
    }

    return rewardIter;
}


} //end of namespace
