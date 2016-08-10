
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#include<Core/util.h>
#include<Core/array.h>

#include <assert.h>

#include "Environment.h"


using namespace mlr;

namespace mdp {


Environment::Environment(arr start)
{
    startState = start;
    currentState = startState;
}

//uint Environment::getStateDim()
//{
//    return currentState.d0;
//}

//void Environment::resetState()
//{
//    currentState = startState;
//}


} //end of namespace
