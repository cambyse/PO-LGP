
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#include<Core/util.h>
#include<Core/array.h>
#include "environment.h"
#include <assert.h>


using namespace mlr;

namespace mdp {




/*
virtual environment class
*/





ENVIRONMENT::ENVIRONMENT(int stateDim, int actionDim, double discount)
    :  StateDim(stateDim), ActionDim(actionDim), Discount(discount)
    {
        assert(discount > 0 && discount <= 1);
        assert(actionDim> 0);
    }



ENVIRONMENT::~ENVIRONMENT()
{
}
arr ENVIRONMENT::GetDomains() const
{
    return Domains;
}



CANNON::CANNON(int stateDim, int actionDim, double discount)
{
    ActionDim = actionDim;
    StateDim = stateDim;
    Discount = discount;

}


bool CANNON::forwardDynamics(arr action, arr state, arr &nxtState, double &reward) const
{
    action(0) = MIN(action(0),MT_PI/2);
    action(0) = MAX(action(0),0.0);
    action(1) = MIN(action(1),10.);
    action(1) = MAX(action(1),0.0);

    //a(0) = a(0) * MT_PI/2;
    //a(1) = a(1) * 6.5;

    nxtState.resize(state.d0);
    nxtState.setZero();


    // Random states currS(d, wind), action(alpha, velocity)
    double m = 1.; //weight of the ball (kg)
    // Compute the cost (-20 time squared distance)
    //time t = 2 * v * sin(alpha) / g;

    double t = (2./9.8)*(action(1)*sin(action(0)));
    // Measure the next distance = t * (v*cos(alpha) + (F*t)/(2*m))
    double distance = action(1)*cos(action(0))*t + state(1)*t*t/(2.*m);
    // Measure cost

    //reward = exp( -1.0*(distance - state(0))*(distance - state(0)));
    reward = -20.0*(distance - state(0))*(distance - state(0));

    return true;
}
void CANNON::getTrajectory(arr actions, arr states, arr &obs, arr &reward, uint horizon) const
{


}


void CANNON::getStartState(arr &sState) const
{
    sState.resize(2);
    //sample a starting state (for only CANNON)
    sState(0) = mlr::rnd.uni(0,10.);
    sState(1) = mlr::rnd.uni(0,1.0);
}





}
