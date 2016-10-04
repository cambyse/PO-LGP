
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_cartPole_Filter_H
#define MDP_cartPole_Filter_H


#include <iostream>
#include <stdint.h>
#include <cstring>

#include <Core/util.h>
#include <Core/array.h>

#include "Filter.h"


namespace mdp {


class cartPole_Filter: public Filter{

private:
    arr observationHistory;
    arr startFeature;
    arr currentFeature;

public:
    //Constructor and destructor
    cartPole_Filter();
    cartPole_Filter(arr start);
    ~cartPole_Filter();

    //Characteristic functions
    void reset();
    void saveObservation(const arr& observation);
    void clearHistory();
    void computeFeature();
    void computeFeature_PO(); //Partial observable MDP
    arr getFeature();

};


}

#endif
