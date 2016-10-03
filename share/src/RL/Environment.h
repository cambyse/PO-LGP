
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_Environment_h
#define MDP_Environment_h


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>


namespace mdp {


class Environment{

protected:
    arr startState;
    arr currentState;
    uint controlType;
    uint obsMDP;

public:
    //Constructor and destructor
    Environment() = default;
    Environment(arr start, uint control, uint observ);
    virtual ~Environment() {} //This is a polymorphic class, pointer Base = Child class.

    //This class is implemented as an interface => pure virtual functions.
    //Inner access
    virtual uint getStateDim() = 0;
    virtual uint getControlType() = 0;
    virtual uint getObsType() = 0;

    //Characteristic functions
    virtual void resetState() = 0;
    virtual bool transition(arr& observation, double& reward, const arr& action) = 0;

};


}

#endif

