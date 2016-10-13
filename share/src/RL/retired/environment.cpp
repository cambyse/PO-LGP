
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




//////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

CARTPOLE::CARTPOLE(int stateDim, int actionDim, double discount)
{
    ActionDim = actionDim;
    StateDim = stateDim;
    Discount = discount;

    GRAVITY =  9.8;
    MASSCART =  1.0;
    MASSPOLE  = 0.1;
    TOTAL_MASS =  (MASSPOLE + MASSCART);
    LENGTH =  0.5;		  /* actually half the pole's length */
    POLEMASS_LENGTH  = (MASSPOLE * LENGTH);
    FORCE_MAG =  10.0;
    TAU  = 0.02;		  /* seconds between state updates */
    FOURTHIRDS =  1.3333333333333;

    one_degree  = 0.0174532;	/* 2pi/360 */
    six_degrees = .1047192;
    twelve_degrees =  0.2094384;
    fifty_degrees = 0.87266;




}


bool CARTPOLE::forwardDynamics(arr action, arr state, arr &nxtState, double &reward) const
{

    ///////////////////////////////////////////
    /// This Cart-Pole simulation is from Sutton's RL book
    /*** Parameters for simulation ***/

    nxtState.resize(state.d0);
    nxtState.setZero();

    double x, x_dot, theta, theta_dot;
    double force, costheta, sintheta;
    x     = state(0);
    x_dot = state(1);
    theta     = state(2);
    theta_dot = state(3);

    nxtState.resize(state.d0);


    //continuous
    force = action(0);
     force = (force>FORCE_MAG)? FORCE_MAG : force;
     force = (force<-FORCE_MAG)? -FORCE_MAG : force;

    costheta = cos(theta);
    sintheta = sin(theta);

    double temp = (force + POLEMASS_LENGTH * theta_dot * theta_dot * sintheta) / TOTAL_MASS;

    double thetaacc = (GRAVITY * sintheta - costheta* temp) / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta / TOTAL_MASS));

    double xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;

/*** Update the four state variables, using Euler's method. ***/

    nxtState(0)  = x     + TAU * x_dot + 0.01*mlr::rnd.gauss();
    nxtState(1)  = x_dot + TAU * xacc ;//+ 0.02*mlr::rnd.gauss();
    nxtState(2)  = theta + TAU * theta_dot + 0.001*mlr::rnd.gauss();
    nxtState(3)  = theta_dot + TAU * thetaacc;// + 0.02*mlr::rnd.gauss();


    if (nxtState(0) < -2.4 || nxtState(0) > 2.4  || nxtState(2) < -twelve_degrees || nxtState(2) > twelve_degrees)
    {
        reward = -1.; /* to signal failure */ //the pole goes beyond the balancing region.
        return true;
    }else{
        reward = 1.0; //still balancing
        return false;
    }


}
void CARTPOLE::getTrajectory(arr actions, arr states, arr &obs, arr &reward, uint horizon) const
{


}


void CARTPOLE::getStartState(arr &sState) const
{
    sState.resize(4);
    sState.setZero();
}


}
