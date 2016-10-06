
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

cartPole_Pol::cartPole_Pol(uint actDim, uint polDim)
{
//    mlr::useLapack = true;
    actionDim = actDim;
    //actionNum = aNum;
    assert(actionDim > 0);

    policyDim = polDim;
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


void cartPole_Pol::sampleAction(arr& action, const arr& currentFeature, const arr& theta)
{
    arr thetaTemp = theta;
    thetaTemp.reshape(actionDim, policyDim);
    arr mean;

    mean = thetaTemp * currentFeature;
    action = mean + sqrt(variance) * mlr::rnd.gauss();
}


void cartPole_Pol::sampleAction_ver1(arr& action, const arr& currentFeature, const arr& theta)
{
    arr thetaTemp = theta;
    thetaTemp.reshape(actionDim, policyDim);
    arr mean;

    for(uint i=0; i<actionDim; i++)
    {
        mean = ~thetaTemp[i] * currentFeature;
        action(i) = mean(0) + sqrt(variance(i)) * mlr::rnd.gauss();
    }
}


void cartPole_Pol::gradLogPol(arr& gradLog, const arr& agentFeature, const arr& theta, const arr& action)
{
    arr thetaTemp = theta;
    thetaTemp.reshape(actionDim, policyDim);
    //gradLog.resizeAs(thetaTemp);

    gradLog = -(thetaTemp * agentFeature - action) * ~agentFeature;
    for(uint index=0; index<thetaTemp.d0; index++)
    {
        gradLog[index] /= variance(index);
    }
}


void cartPole_Pol::gradLogPol_ver1(arr& gradLog, const arr& agentFeatures, const arr& theta, const arr& actions)
{
    //agentFeatures is a matrix of features! (of a whole rollout)
    arr thetaTemp = theta;
    thetaTemp.reshape(actionDim, policyDim);
    gradLog.resizeAs(thetaTemp);
    gradLog.setZero();

    for(uint i=0; i<agentFeatures.d0; i++)
    {
        arr aux = agentFeatures[i];
        for(uint j=0; j<actionDim; j++)
        {
            gradLog[j] -= (~thetaTemp[j] * aux - actions(i,j)) * ~aux / variance(j);
        }
    }
}


} //end of namespace
