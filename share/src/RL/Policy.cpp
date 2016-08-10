
#include <Core/util.h>
#include <Core/array.h>

#include <assert.h>
#include <float.h>

#include "Policy.h"

/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


using namespace mlr;

namespace mdp {


//Policy::Policy() {}

Policy::Policy(uint aDim, uint pDim)
{
//    mlr::useLapack = true;
    actionDim = aDim;
    //actionNum = aNum;
    assert(actionDim > 0);

    policyDim = pDim;
}

//Policy::~Policy() {}

uint Policy::getActionDim()
{
    return actionDim;
}

uint Policy::getPolicyDim()
{
    return policyDim;
}

//void Policy::sampleAction(arr& currentAgentObs, const arr& theta, arr& action) {}

//void Policy::gradLogPol(arr& agentObservations, arr& theta, arr& actions, arr& gradLog) {}

/*uint Policy::getActionNum()
{
   return actionNum;
}*/


} //end of namespace
