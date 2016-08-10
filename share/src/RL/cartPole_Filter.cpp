
#include <Core/util.h>
#include <Core/array.h>
#include <float.h>

#include "cartPole_Filter.h"


/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


using namespace mlr;

namespace mdp {


cartPole_Filter::cartPole_Filter()
{
    perceptionHistory.clear();
    currentEstimate.clear();
}

cartPole_Filter::~cartPole_Filter() {}

void cartPole_Filter::savePerception(const arr& perception)
{
    perceptionHistory.append(~perception);
}

void cartPole_Filter::clearHistory()
{
    perceptionHistory.clear();
}

void cartPole_Filter::computeEstimate()
{
    uint lastRow;
    lastRow = perceptionHistory.d0 - 1;
    currentEstimate = perceptionHistory[lastRow];
}

arr cartPole_Filter::getObsEstimate()
{
    return currentEstimate;
}


} //end of namespace
