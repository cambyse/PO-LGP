
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_cartPole_Env_h
#define MDP_cartPole_Env_h


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>

#include "Environment.h"


namespace mdp {


class cartPole_Env: public Environment{

protected:
    arr startState;
    arr currentState;
    uint controlType;
    uint obsMDP;

public:
    //Constructor and destructor
    cartPole_Env() {}
    cartPole_Env(arr start, uint control, uint observ);
    ~cartPole_Env() {}

    //Inner access
    uint getStateDim();
    uint getControlType();
    uint getObsType();

    //Characteristic functions
    void resetState();
    bool transition(arr& observation, double& reward, const arr& action);

};


}

#endif

