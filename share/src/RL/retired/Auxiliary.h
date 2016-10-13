
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_Auxiliary_h
#define MDP_Auxiliary_h


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>

#include "PolicySearch.h"

namespace mdp {


void gradient_FD(PolicySearch obj, arr x);
void myCheckGradient(PolicySearch obj, arr x);


}

#endif

