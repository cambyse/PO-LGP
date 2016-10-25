
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_Filter_H
#define MDP_Filter_H


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>


namespace mdp {


class Filter{

private:
    arr observationHistory;
    arr startFeature;
    arr currentFeature;

public:
    //Constructor and destructor
    Filter();
    Filter(arr start);
    virtual ~Filter() {}

    //Characteristic functions
    virtual void reset() = 0;
    virtual void saveObservation(const arr& observation);
    virtual void clearHistory() = 0;
    virtual void computeFeature() = 0;
    virtual void computeFeature_PO() = 0;
    virtual arr getFeature() = 0;

};


}

#endif
