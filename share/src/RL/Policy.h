
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_Policy_H
#define MDP_Policy_H


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>


namespace mdp {


class Policy{

protected:
    uint actionDim;
    //uint actionNum;
    uint policyDim;

public:
    //Constructor and destructor
    Policy() = default;
    Policy(uint actDim, uint polDim);
    virtual ~Policy() {}

    //Inner access
    virtual uint getActionDim();
//    virtual uint getActionNum();
    virtual uint getPolicyDim();

    //Characteristic functions
    virtual void sampleAction(arr& action, const arr& currentFeature, const arr& theta) = 0;
    virtual void gradLogPol(arr& gradLog, const arr& agentFeature, const arr& theta, const arr& action) = 0;

    //Modified functions
    virtual void sampleAction_ver1(arr& action, const arr& currentFeature, const arr& theta) = 0;
    virtual void gradLogPol_ver1(arr& gradLog, const arr& agentFeatures, const arr& theta, const arr& actions) = 0;

};


}

#endif
