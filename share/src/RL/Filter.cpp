
#include <Core/util.h>
#include <Core/array.h>
#include <float.h>

#include "Filter.h"


/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


using namespace mlr;

namespace mdp {


#if 0
  Filter::Filter()
{
    perceptionHistory.clear();
    currentEstimate.clear();
}

//Filter::~Filter() {}

void Filter::savePerception(const arr& perception)
{
    perceptionHistory.append(~perception);
}
#endif

//void Filter::computeEstimate()
//{
//    currentEstimate.setZero();
//}

//arr Filter::getObsEstimate()
//{
//    return currentEstimate;
//}


} //end of namespace
