
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
    observationHistory.clear(); //Working with append, so clear to be sure there's nothing at the beginning.
    currentFeature.clear();
}


cartPole_Filter::cartPole_Filter(arr start)
{
    observationHistory.clear();
    currentFeature.clear();
    startFeature = start;
    currentFeature = startFeature;
}


cartPole_Filter::~cartPole_Filter() {}


void cartPole_Filter::reset()
{
    currentFeature = startFeature;
}


void cartPole_Filter::saveObservation(const arr& observation)
{
    observationHistory.append(~observation);
}


void cartPole_Filter::clearHistory()
{
    observationHistory.clear();
}


void cartPole_Filter::computeFeature()
{
    uint lastRow;
    lastRow = observationHistory.d0 - 1;
    currentFeature = observationHistory[lastRow];
}


void cartPole_Filter::computeFeature_PO()
{
    uint windowSize = 5;
    uint dim0 = observationHistory.d0;
    uint dim1 = observationHistory.d1;
    uint startRow;

    if (dim0 >= windowSize) //if enough observations available, retain only the last 5
    {
        startRow = dim0 - windowSize;
        currentFeature = observationHistory.sub(startRow, -1, 0, -1);
    }
    else //if not enough observations available, complete the rest with zeros
    {
        currentFeature = zeros(windowSize, dim1);
        currentFeature({0, dim0-1}) = observationHistory;
    }

    currentFeature.reshape(windowSize * dim1); //vector
}


arr cartPole_Filter::getFeature()
{
    return currentFeature;
}


} //end of namespace
