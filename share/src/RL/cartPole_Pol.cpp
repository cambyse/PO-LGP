
#include<Core/util.h>
#include<Core/array.h>

#include <assert.h>
#include <float.h>

#include "cartPole_Pol.h"

/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


using namespace mlr;

namespace mdp {


cartPole_Pol::cartPole_Pol() {}

cartPole_Pol::cartPole_Pol(uint aDim, uint pDim)
{
//    mlr::useLapack = true;
    actionDim = aDim;
    //actionNum = aNum;
    assert(actionDim > 0);

    policyDim = pDim;
    assert(policyDim > 0);

    //This exploration parameters can be changed in file "MT.cfg"
    double var1 = mlr::getParameter<double>("var1",0.5);
    variance = ARR(var1);
}

cartPole_Pol::~cartPole_Pol() {}

uint cartPole_Pol::getActionDim()
{
    return actionDim;
}

uint cartPole_Pol::getPolicyDim()
{
    return policyDim;
}

/*uint cartPole_Pol::getActionNum()
{
   return actionNum;
}*/


void cartPole_Pol::sampleAction(arr& currentAgentObs, const arr& theta, arr& action)
{
    currentAgentObs.resize(policyDim);
    action.resize(actionDim);
    arr thetaTemp;
    thetaTemp = theta;
//    thetaTemp.reshape(actionDim, currentAgentObs.d0);
    thetaTemp.reshape(actionDim, policyDim);
    arr mean;

    for(uint i=0; i<actionDim; i++)
    {
        mean = ~thetaTemp[i] * currentAgentObs;
        action(i) = mean(0) + sqrt(variance(i)) * mlr::rnd.gauss();
    }
}


void cartPole_Pol::gradLogPol(arr& agentObservations, arr& theta, arr& actions, arr& gradLog)
{
    //agentObservations is a matrix of observations! (of a whole rollout)
    arr thetaTemp = theta;
//    thetaTemp.reshape(actionDim, agentObservations.d1);
    thetaTemp.reshape(actionDim, policyDim);
    gradLog.resizeAs(thetaTemp);
    gradLog.setZero();

    for(uint i=0; i<agentObservations.d0; i++)
    {
        for(uint j=0; j<actionDim; j++)
        {
            //arr aux = ~thetaTemp[j] * agentObservations[i];
            //aux = actions[i](j);
            gradLog[j] -= (~thetaTemp[j] * agentObservations[i] - actions(i,j)) * ~agentObservations[i] / variance(j);
        }
    }
}


} //end of namespace
