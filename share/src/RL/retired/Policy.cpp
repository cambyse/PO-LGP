
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

Policy::Policy(uint actDim, uint polDim)
{
//    mlr::useLapack = true;
    actionDim = actDim;
    //actionNum = aNum;
    assert(actionDim > 0);

    policyDim = polDim;
    assert(policyDim > 0);
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

//void Policy::sampleAction(arr& action, const arr& currentFeature, const arr& theta) {}

//void Policy::gradLogPol(arr& gradLog, const arr& agentFeature, const arr& theta, const arr& action) {}

/*uint Policy::getActionNum()
{
   return actionNum;
}*/


} //end of namespace
