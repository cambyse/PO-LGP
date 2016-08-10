
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
    Policy(uint aDim, uint pDim);
    virtual ~Policy() {}

    //Inner access
    virtual uint getActionDim();
//    virtual uint getActionNum();
    virtual uint getPolicyDim();

    //Characteristic functions
    virtual void sampleAction(arr& currentAgentObs, const arr& theta, arr& action) = 0;
    virtual void gradLogPol(arr& agentObservations, arr& theta, arr& actions, arr& gradLog) = 0;

};


}

#endif
