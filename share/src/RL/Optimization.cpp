
#include<Core/util.h>
#include<Core/array.h>

#include "Optimization.h"

/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


using namespace mlr;


namespace mdp {


Optimization::Optimization(bool flag, double min, double max)
{
    limitFlag = flag;
    minL = min;
    maxL = max;
}


Optimization::~Optimization() {}


void Optimization::RPROP(arr& step, const arr& gradNew, arr& gradOld, arr& theta)
{
    theta.reshapeAs(gradNew);

    for(uint i=0; i<gradNew.d0; i++)
    {
        for(uint index=0; index<gradNew.d1; index++)
        {
            if(gradNew(i,index) * gradOld(i,index) > 0.0){
                step(i,index) = 1.2 * step(i,index);
                theta(i,index) = theta(i,index) + step(i,index) * sign(gradNew(i,index));
                gradOld(i,index) = gradNew(i,index);
            }
            else if(gradNew(i,index) * gradOld(i,index) < 0.0){
                step(i,index) = 0.5 * step(i,index);
                theta(i,index) = theta(i,index) + step(i,index) * sign(gradNew(i,index));
                gradOld(i,index) = 0;
            }
            else{
                theta(i,index) = theta(i,index) + step(i,index) * sign(gradNew(i,index));
                gradOld(i,index) = gradNew(i,index);
            }

            //Check if between the limits
            if(limitFlag){
                if(step(i,index) > maxL) step(i,index) = maxL;
                if(step(i,index) < minL) step(i,index) = minL;
            }
        }
    }
    theta.reshape(gradNew.d0*gradNew.d1);
}


} //end of namespace
