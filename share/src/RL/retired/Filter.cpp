
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
    observationHistory.clear();
    currentFeature.clear();
}

Filter::Filter(arr start)
{
    observationHistory.clear();
    currentFeature.clear();
    startFeature = start;
    currentFeature = startFeature;
}

//Filter::~Filter() {}

void Filter::saveObservation(const arr& observation)
{
    observationHistory.append(~observation);
}
#endif

//void Filter::computeFeature()
//{
//    currentEstimate.setZero();
//}

//arr Filter::getFeature()
//{
//    return currentEstimate;
//}


} //end of namespace
