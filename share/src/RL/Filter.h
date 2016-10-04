
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
//    arr perceptionHistory;
//    arr currentEstimate;

public:
    //Constructor and destructor
//    Filter();
    virtual ~Filter() {}

    //Characteristic functions
    virtual void savePerception(const arr& perception){ NIY; }
    virtual void clearHistory() = 0;
    virtual void computeEstimate(){ NIY; }
    virtual arr getObsEstimate(){ NIY; }

    virtual uint getFeatureDim(){ NIY; }
    virtual arr filterUpdate(const arr& action, const arr& perception){ NIY; }

};


}

#endif
