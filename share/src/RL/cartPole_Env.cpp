
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


#include<Core/util.h>
#include<Core/array.h>

#include <assert.h>

#include "cartPole_Env.h"

/*** Parameters for cart pole simulation ***/
#define GRAVITY 9.8
#define MASSCART 1.0
#define MASSPOLE 0.1
#define TOTAL_MASS (MASSPOLE + MASSCART)
#define LENGTH 0.5		  /* actually half the pole's length */
#define POLEMASS_LENGTH (MASSPOLE * LENGTH)
#define FORCE_MAG 10.0
#define TAU 0.02		  /* seconds between state updates */
#define FOURTHIRDS 1.3333333333333

#define one_degree 0.0174532	/* 2pi/360 */
#define six_degrees 0.1047192
#define twelve_degrees 0.2094384
#define fifty_degrees 0.87266


using namespace mlr;

namespace mdp {


cartPole_Env::cartPole_Env(arr start, uint control, uint observ)
{
    startState = start;
    currentState = startState;
    controlType = control;
    obsMDP = observ;
}


uint cartPole_Env::getStateDim()
{
    return currentState.d0;
}


uint cartPole_Env::getControlType()
{
    return controlType;
}


uint cartPole_Env::getObsType()
{
    return obsMDP;
}


void cartPole_Env::resetState()
{
    currentState = startState;
}


bool cartPole_Env::transition(arr& observation, double& reward, const arr& action)
{
    observation.clear(); //Using function "cat" to obtain the observation. Clear is needed.
    bool terminal = false;
    double x, x_dot, theta, theta_dot;
    double force, costheta, sintheta;
    x     = currentState(0);
    x_dot = currentState(1);
    theta     = currentState(2);
    theta_dot = currentState(3);

    //Continuous action, but deterministic (force is just min or max) -> bag bag control
    //Implement also stochastic control (don't forget clipping - force within the interval)
    if (controlType == 1)
    {
        force = action(0);
        force = MIN(force, FORCE_MAG);
        force = MAX(force, -FORCE_MAG);
    }
    else if (controlType == 0) force = (action(0) > 0) ? FORCE_MAG : -FORCE_MAG;
    else cout<<"This control type isn't defined in this implementation."<<endl;

    costheta = cos(theta);
    sintheta = sin(theta);

    double temp = (force + POLEMASS_LENGTH * theta_dot * theta_dot * sintheta) / TOTAL_MASS;

    double thetaacc = (GRAVITY * sintheta - costheta* temp) / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta / TOTAL_MASS));

    double xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;

    /*** Update the four state variables, using Euler's method. ***/

    currentState(0)  = x     + TAU * x_dot + 0.01 * mlr::rnd.gauss();
    currentState(1)  = x_dot + TAU * xacc ; //+ 0.02 * mlr::rnd.gauss();
    currentState(2)  = theta     + TAU * theta_dot + 0.001 * mlr::rnd.gauss();
    currentState(3)  = theta_dot + TAU * thetaacc; // + 0.02 * mlr::rnd.gauss();

    if (obsMDP == 1) observation = currentState;
    else
    {
        arr aux = ARR(currentState(3));
        observation = cat(observation, action, aux);
    }

    if (currentState(0) < -2.4 || currentState(0) > 2.4  || currentState(2) < -twelve_degrees || currentState(2) > twelve_degrees)
    {
        reward = -1.; /* to signal failure */ //the pole goes beyond the balancing region.
        terminal = true;
    }
    else
        reward = 1.0; //still balancing

    return terminal;
}


} //end of namespace
