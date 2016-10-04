
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_Environment_h
#define MDP_Environment_h


#include<Core/array.h>


namespace mdp {

class Environment{

protected:
    arr startState;  //mt: must not be part of virtual class, only concrete class
    arr currentState;  //mt: must not be part of virtual class, only concrete class

public:
    //Constructor and destructor
    Environment() = default;
    Environment(arr start);
    virtual ~Environment() {} //This is a polymorphic class, pointer Base = Child class.

    //This class is implemented as an interface => pure virtual functions.
    //Inner access
    virtual uint getStateDim() = 0;

    //Characteristic functions
    virtual void resetState() = 0;
    virtual bool transition(const arr& action, arr& perception, double& reward) = 0; //mt: swap argument orders //mt: bool indicates terminal state?

};


}

#endif

