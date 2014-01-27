#include <Core/array.h>
#include <Ors/ors.h>

arr spline(const arr&, double);
arr spline_vel(const arr&, double);

struct FastControllerInterface{
  //-- parameterize the fast controller
  void setConfigurationPrior(const arr& q, const arr& qdot, double weight);
  void setEndeffPosition(const arr& x, const arr& xdot, double weight);
  void setEndeffForce(const arr& forceVector, double weight);

  //-- initiate a step and wait for its completion
  void makeStep();
  void waitForStep();

  //-- access feedback from the lower level
  void getConfiguration(arr& q, arr& qdot);
  void getEndeffPosition(arr& x, arr& xdot);
  void getEndeffForce(arr& forceVector);
};

void dualExecution(const arr& x, const arr& y, const arr& dual, ors::KinematicWorld& world);


//struct OuterController{
//  FastControllerInterface &C;
//  arr q_reference;  ///< configuration space trajectory
//  arr x_reference;  ///< endeffector space trajectory
//  arr dual_reference; ///< dual trajectory

//  arr g_shift, g_normal;  ///< current constraint offset estimate
//  double phase;  ///< current phase within the plan

//  void executePlan(){
//    while(phase<FINISH){
//      step_PseudoCode();
//    }
//  }

//  void step_PseudoCode(){
//    arr q,qdot,x,xdot,f;
//    C.getConfiguration(q, qdot);
//    C.getEndeffPosition(x, xdot);
//    C.getEndeffForce(f);

//    //-- recalibrate $g$ based on feedback

//    //based on f: recalibrate the constraint, if necessary
//    //modify g_shift

//    //-- decide on the phase progression

//    //naively, phase += 1.; (progress as planned)
//    //but based on feedback phase might progress slower/faster

//    //-- decide on desireds

//    q = spline(q_reference, phase); //interpolatesup the reference at the phase
//    qdot = spline_vel(q_reference, phase); //interpolatesup the reference at the phase

//    x = spline(x_reference, phase);
//    x += g_shift; //modify the endeff reference by the estimated g_shift
//    xdot = spline_vel(x_reference, phase);

//    double lambda = spline(dual_reference, phase);
//    if(lambda>0.){ //we have a desired force
//      f = - CONSTANT * g_normal;
//    }else{
//      f = 0.;
//    }

//    //-- send to controller
//    C.waitForStep();
//    C.setConfigurationPrior(q, qdot, lowWeigt);
//    C.setEndeffPosition(x, xdot, midWeight);
//    C.setEndeffForce(f, highWeight);
//    C.makeStep(); //don't wait for done here
//  }
//};
