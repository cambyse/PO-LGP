
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_cartPole_Pol_H
#define MDP_cartPole_Pol_H


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>

#include "Policy.h"

namespace mdp {


class cartPole_Pol: public Policy{

protected:
    uint actionDim;
    //uint actionNum;
    uint policyDim;

public:
    //Other variable members
    arr variance;

    //Constructor and destructor
    cartPole_Pol();
    cartPole_Pol(uint actDim, uint polDim);
    ~cartPole_Pol();

    //Inner access
    uint getActionDim();
    //uint getActionNum();
    uint getPolicyDim();

    //Characteristic functions
    void sampleAction(arr& action, const arr& currentFeature, const arr& theta);
    void gradLogPol(arr& gradLog, const arr &agentFeature, const arr &theta, const arr &action);

    //Modified functions
    void sampleAction_ver1(arr& action, const arr& currentFeature, const arr& theta);
    void gradLogPol_ver1(arr& gradLog, const arr &agentFeatures, const arr &theta, const arr &actions);
};


}

#endif
