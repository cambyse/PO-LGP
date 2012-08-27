#ifndef MT_socNew_h
#define MT_socNew_h

#include "util.h"
#include "array.h"
#include "kOrderMarkovProblem.h"

extern uint countMsg, countSetq;

struct ControlledSystem {
  // access general problem information
  virtual uint get_T() = 0;  ///< total time steps of the trajectory (that's time_slices - 1)
  virtual uint get_xDim() = 0; //the dimensionality of $x_t$
  virtual uint get_uDim() = 0;                 ///< dimensionality of control
  virtual uint get_phiDim(uint t) = 0; //the dimensionality of the task vector $\phi_t$
  virtual void get_x0(arr& x0) = 0;          ///< start joint configuration and velo
  virtual bool isKinematic() = 0;

  // access dynamics and task vector for given state x
  //NOTE: arguments may be @NoArr@ if they're not needed!!
  virtual void setx(const arr& x) = 0;
  virtual arr& getx() = 0;             ///< get the current x!
  virtual void getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a, arr& B, arr& Bt, arr& Q, uint t) = 0;
  virtual void getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t){
    getDynamics(A, NoArr, NoArr, NoArr, a, B, NoArr, Q, t); }
  virtual void getControlCosts(arr& H, arr& Hinv, uint t) = 0;              ///< dynamic control cost metric: step cost = u^T H u, with H = tau*H_rate where tau is step size
  virtual void getTaskCosts(arr& phi, arr& phiJ, uint t) = 0; ///< the general vector and its Jacobian
  virtual double getTaskCosts(arr& R, arr& r, uint t, double* rhat);

  // display and info
  virtual void displayCurrentState(const char* title=NULL, bool pause=false, bool reportOnTasks=false) = 0;
  virtual void getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names) = 0;

  std::ostream *os;
  struct OpenGL *gl;
};


void getTransitionCostTerms(ControlledSystem& sys, bool dynamic, arr& Psi, arr& J0, arr& J1, const arr& x0, const arr& x1, uint t);
double analyzeTrajectory(ControlledSystem& sys, const arr& x, bool plot, std::ostream* os);
void displayTrajectory(ControlledSystem& sys, const arr& x, const arr *Binv, int steps, const char *tag);

struct KOrderMarkovFunction_ControlledSystem:KOrderMarkovFunction {
  ControlledSystem *sys;

  KOrderMarkovFunction_ControlledSystem(ControlledSystem& _sys):sys(&_sys){}

  uint get_T(){ return sys->get_T(); }
  uint get_k(){ return 1; }
  uint get_n(){ return sys->get_xDim(); }
  uint get_m(uint t){ return sys->get_phiDim(t) + sys->get_xDim(); }
  void phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
    arr x0(x_bar,0);
    arr x1(x_bar,1);

    sys->setx(x0);
    
    //dynamics
    arr J0, J1;
    getTransitionCostTerms(*sys, true, phi, J0, J1, x0, x1, t);
    if(&J){
      J.resize(J0.d0, J0.d1+J1.d1);
      J.setMatrixBlock(J0,0,0);
      J.setMatrixBlock(J1,0,J0.d1);
    }

    //tasks
    arr _phi, _J;
    sys->getTaskCosts(_phi, _J, t);
    _J.insColumns(_J.d0, x1.N);
    for(uint i=0;i<_J.d0;i++) for(uint j=x0.N;j<_J.d1;j++) _J(i,j) = 0.;
    phi.append(_phi);
    if(&J) J.append(_J);
  }
};

#endif
