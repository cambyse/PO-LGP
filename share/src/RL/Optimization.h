
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#ifndef MDP_Optimization_H
#define MDP_Optimization_H


#include <iostream>
#include <stdint.h>
#include <cstring>

#include<Core/util.h>
#include<Core/array.h>


namespace mdp {


class Optimization{

protected:
    //Specify the limits for the step
    bool limitFlag;
    double minL, maxL;

public:
    //Constructor and destructor
    Optimization() {};
    Optimization(bool flag, double min = .0, double max = .0);
    ~Optimization();

    //Characteristic functions
    void RPROP(arr& step, const arr& gradNew, arr& gradOld, arr& theta);

};


}

#endif
