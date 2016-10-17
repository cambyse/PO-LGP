
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/



#ifndef MDP_environment_h
#define MDP_environment_h



#include <iostream>
#include <stdint.h>
#include <cstring>

#include<Core/util.h>
#include<Core/array.h>

#define MT_PI 3.14159265358979






namespace mdp {




////////////////////////////////////////////////////////////////////////////////
class ENVIRONMENT{

public:

    //ENVIRONMENT(double discount = 1.0);
    ENVIRONMENT(int stateDim=1, int actionDim=1, double discount = 1.0);
    virtual ~ENVIRONMENT();


    double GetDiscount() const { return Discount; }
    int GetNumActions()  const { return NumActions; }
    int GetActionDim()  const { return ActionDim; }
    int GetStateDim()  const { return StateDim; }
    virtual void getStartState(arr &sState) const = 0;



    virtual bool forwardDynamics(arr action, arr state, arr &nxtState, double &reward) const = 0;
    virtual void getTrajectory(arr actions, arr states, arr &obs, arr &reward, uint horizon) const = 0;

    arr GetDomains() const;

protected:
    int NumActions;
    int ActionDim;
    int StateDim;
    double Discount;
    arr Domains;
    arr currState;


};


//CANNON
class CANNON: public ENVIRONMENT{

public:

   CANNON(int stateDim=1, int actionDim=1, double discount = 1.0);
   ~CANNON(){}

   virtual bool forwardDynamics(arr action, arr state, arr &nxtState, double &reward) const;
   virtual void getTrajectory(arr actions, arr states, arr &obs, arr &reward, uint horizon) const ;
   virtual void getStartState(arr &sState) const;

};



//CART-POLE
class CARTPOLE: public ENVIRONMENT{
private:
   double GRAVITY,MASSCART,  MASSPOLE, TOTAL_MASS, LENGTH ;		  /* actually half the pole's length */
   double POLEMASS_LENGTH , FORCE_MAG ,TAU  , FOURTHIRDS, one_degree, six_degrees;
   double twelve_degrees, fifty_degrees;

public:

   CARTPOLE(int stateDim=4, int actionDim=1, double discount = 1.0);
   ~CARTPOLE(){}

   virtual bool forwardDynamics(arr action, arr state, arr &nxtState, double &reward) const;
   virtual void getTrajectory(arr actions, arr states, arr &obs, arr &reward, uint horizon) const ;
   virtual void getStartState(arr &sState) const;

};



} // end of namespace




#endif
