


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
    step = ones(1,Pol.getPolicyDim()) * 0.7;
}


PolicySearch::~PolicySearch() {}


uint PolicySearch::getHorizon()
{
    return Horizon;
}


uint PolicySearch::getNumEps()
{
    return NumEps;
}


Policy* PolicySearch::getPolicy()
{
    return &Pol;
}


double PolicySearch::rollout(const arr& theta)
{
     Env.resetState();
     Fil.clearHistory();
     Fil.reset();
     arr currentAgentFeature = Fil.getFeature();
     arr action;
     arr observation;

     double reward;
     double totalReturn = 0.0;

     bool terminal;

     for(uint t=0; t<getHorizon(); t++)
     {
        Pol.sampleAction(action, currentAgentFeature, theta);

        terminal = Env.transition(observation, reward, action);
        Fil.saveObservation(observation);

        if (Env.getObsType() == 1)
            Fil.computeFeature();
        else
            Fil.computeFeature_PO();
        currentAgentFeature = Fil.getFeature();

        totalReturn += pow(Discount,t) * reward;

        if(terminal) break;
     }

     return totalReturn;
}


double PolicySearch::just_rollout(arr& features, arr& actions, arr& rewards, const arr& theta, uint numSteps)
{
     features.clear();
     actions.clear();
     rewards.clear();

     Env.resetState();
     Fil.clearHistory();
     Fil.reset();
     arr currentAgentFeature = Fil.getFeature();
     arr action;
     arr observation;

     double reward;
     double totalReturn = 0.0;  

     bool terminal;

     for(uint t=0; t<numSteps; t++)
     {
        Pol.sampleAction(action, currentAgentFeature, theta);
        features.append(~currentAgentFeature);

        //Append as a line - in actions, a row is an action
        actions.append(~action);

        terminal = Env.transition(observation, reward, action);
        Fil.saveObservation(observation);
        rewards.append(reward);

        if (Env.getObsType() == 1)
            Fil.computeFeature();
        else
            Fil.computeFeature_PO();
        currentAgentFeature = Fil.getFeature();

        totalReturn += pow(Discount,t) * reward;

        if(terminal) break;
     }

     return totalReturn;
}


double PolicySearch::grad_rollout(arr& gradLogRet, arr& actions, arr& rewards, const arr& theta, uint numSteps)
{
    gradLogRet.resize(Pol.getActionDim(), Pol.getPolicyDim()); //comes as a vector, transform into matrix
    gradLogRet.setZero();
    actions.clear();
    rewards.clear();

    Env.resetState();
    Fil.clearHistory();
    Fil.reset();
    arr currentAgentFeature = Fil.getFeature();
    arr gradLog;
    arr action;
    arr observation;

    double reward;
    double totalReturn = 0.0;

    bool terminal;

    for(uint t=0; t<numSteps; t++)
    {
        Pol.sampleAction(action, currentAgentFeature, theta);
        Pol.gradLogPol(gradLog, currentAgentFeature, theta, action); //gradLog is a matrix
        gradLogRet += gradLog; //matrices

        //Append as a line - in actions, a row is an action
        actions.append(~action);

        terminal = Env.transition(observation, reward, action);
        Fil.saveObservation(observation);
        rewards.append(reward);

        if (Env.getObsType())
            Fil.computeFeature();
        else
            Fil.computeFeature_PO();
        currentAgentFeature = Fil.getFeature();

        totalReturn += pow(Discount,t) * reward;

        if(terminal) break;
    }

    return totalReturn;
}


double PolicySearch::updateREINFORCE(arr& gradJ, const arr& theta)
{
    arr actions, rewards;
    arr gradLog, nom, den, b;
    gradLog.resizeAs(gradJ);
    nom.resizeAs(gradJ);
    den.resizeAs(gradJ);
    b.resizeAs(gradJ);

    double Reward, total, average;
    total = .0;

    arr aux;

    for(uint ep=0; ep<NumEps; ep++)
    {
        nom.setZero(); den.setZero();

        for(uint epBase=0; epBase<NumEps; epBase++)
        {
            Reward = grad_rollout(gradLog, actions, rewards, theta, Horizon); //gradLog matrix

            aux = gradLog;
            tensorMultiply(aux, ~gradLog, TUP(1,0));
            nom += aux * Reward;
            den += aux;
        }

        nom /= (double)NumEps;
        den /= (double)NumEps;
        for(uint i=0; i<nom.d0; i++)
            for(uint index=0; index<nom.d1; index++)
                b(i,index) = nom(i,index) / den(i,index);

        Reward = grad_rollout(gradLog, actions, rewards, theta, Horizon);
        total += Reward;

        aux = gradLog;
        tensorMultiply(aux, ~(Reward-b), TUP(1,0));
        gradJ += aux;
    }

    gradJ /= (double)NumEps;
    average = (double) total/NumEps;
    return average;
}


double PolicySearch::updateREINFORCE_ver1(arr& gradJ, const arr& theta)
{
    arr features, actions, rewards;
    arr gradLog, nom, den, b;
    gradLog.resizeAs(gradJ);
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
            Reward = just_rollout(features, actions, rewards, theta, Horizon); //gradLog matrix
            Pol.gradLogPol_ver1(gradLog, features, theta, actions);

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

        Reward = just_rollout(features, actions, rewards, theta, Horizon);
        total += Reward;

        Pol.gradLogPol_ver1(gradLog, features, theta, actions);
        for(uint i=0; i<gradLog.d0; i++)
            for(uint index=0; index<gradLog.d1; index++)
                gradJ(i,index) += gradLog(i,index) * (Reward - b(i,index));
    }

    gradJ /= (double)NumEps;
    gradJ.reshape(Pol.getActionDim(), Pol.getPolicyDim());
    average = (double) total/NumEps;
    return average;
}


arr PolicySearch::runREINFORCE()
{
    arr theta; theta.resize(Pol.getActionDim()*Pol.getPolicyDim()); theta.setZero(); //vector
    arr gradJ; gradJ.resize(Pol.getActionDim(), Pol.getPolicyDim()); //matrix
    arr gradJOld; gradJOld.resizeAs(gradJ); gradJOld.setZero();
    arr rewardIter(NumIter);
    double rewardMean;

    for(uint iter=0; iter<NumIter; iter++)
    {
        gradJ.setZero();
        rewardMean = updateREINFORCE(gradJ, theta);
//        rewardMean = updateREINFORCE_ver1(gradJ, theta);
        Solver.RPROP(gradJOld, theta, step, gradJ);
        cout<<" The expected reward of iteration "<< iter <<" is: "<< rewardMean <<endl;
        rewardIter(iter) = rewardMean;
    }

    return rewardIter;
}


double PolicySearch::updateGPOMDP_ver1(arr& gradJ, const arr &theta)
{
    arr features, actions, rewards;
    arr featMain, actMain, rewMain;
    arr auxFeat, auxAct;
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
        Reward = grad_rollout(features, actions, rewards, theta, Horizon);
        total += Reward;
        featMain = features;
        actMain = actions;
        rewMain = rewards;

        for(uint j=0; j<features.d0; j++)
        {
            auxFeat = featMain.sub(0,j, 0,-1);
            auxAct = actMain.sub(0,j, 0,-1);
            Pol.gradLogPol(gradLog, auxFeat, theta, auxAct);
            gradLogMain = gradLog;

            nom.setZero(); den.setZero();
            for(uint epBase=0; epBase<NumEps; epBase++)
            {
                Reward = grad_rollout(features, actions, rewards, theta, j);
                Pol.gradLogPol(gradLog, features, theta, actions);

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


double PolicySearch::updateGPOMDP(arr& gradJ, const arr& theta)
{
    arr features, actions, rewards;
    arr featMain, actMain, rewMain;
    arr auxFeat, auxAct, aux;
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
        Reward = just_rollout(features, actions, rewards, theta, Horizon);
        total += Reward;
        featMain = features;
        actMain = actions;
        rewMain = rewards;

        for(uint j=0; j<features.d0; j++)
        {
            auxFeat = featMain.sub(0,j, 0,-1);
            auxAct = actMain.sub(0,j, 0,-1);
            Pol.gradLogPol_ver1(gradLog, auxFeat, theta, auxAct);
            gradLogMain = gradLog;

            nom.setZero(); den.setZero();
            for(uint epBase=0; epBase<NumEps; epBase++)
            {
                Reward = grad_rollout(gradLog, actions, rewards, theta, j);
//                Pol.gradLogPol(gradLog, features, theta, actions);

                aux = gradLog;
                tensorMultiply(aux, ~gradLog, TUP(1,0));
                nom += aux * rewMain(j);
                den += aux;
            }

            nom /= (double)NumEps;
            den /= (double)NumEps;
            for(uint i=0; i<nom.d0; i++)
                for(uint index=0; index<nom.d1; index++)
                    b(i,index) = nom(i,index) / den(i,index);

            aux = gradLogMain;
            tensorMultiply(aux, ~(rewMain(j) - b), TUP(1,0));
            gradJ += aux;
        }
    }

    gradJ /= (double)NumEps;
    average = (double) total/NumEps;
    return average;
}


arr PolicySearch::runGPOMDP()
{
    arr theta; theta.resize(Pol.getActionDim()*Pol.getPolicyDim()); theta.setZero();
    arr gradJ; gradJ.resize(Pol.getActionDim(), Pol.getPolicyDim());
    arr gradJOld; gradJOld.resizeAs(gradJ); gradJOld.setZero();
    arr rewardIter(NumIter);
    double rewardMean;

    for(uint iter=0; iter<NumIter; iter++)
    {
        gradJ.setZero();
        rewardMean = updateGPOMDP(gradJ, theta);
        Solver.RPROP(gradJOld, theta, step, gradJ);
        cout<<" The expected reward of iteration "<< iter <<" is: "<< rewardMean <<endl;
        rewardIter(iter) = rewardMean;
    }

    return rewardIter;
}


} //end of namespace
