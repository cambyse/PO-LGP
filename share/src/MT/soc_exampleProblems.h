#ifndef MT_soc_exampleProblems_h
#define MT_soc_exampleProblems_h

#include "socNew.h"

/** \brief an implementation of the SocSystemAbstraction that simulates a
    single 1D point mass on a spring */
struct ControlledSystem_PointMass: ControlledSystem{

  uint T;
  double tau;
  arr x;
  arr x0, x_target;
  double prec;

  ControlledSystem_PointMass();
  virtual ~ControlledSystem_PointMass();

  // implementation of virtuals
  virtual uint get_T(){ return T; }
  virtual uint get_xDim(){ return 2; }
  virtual uint get_uDim(){ return 1; }
  virtual uint get_phiDim(uint t){ NIY }
  virtual void get_x0(arr& x0){ x0 = this->x0; }
  virtual bool isKinematic(){ return false; }

  virtual void setx(const arr& x){ this->x = x; }
  virtual arr& getx(){ return this->x; }
  virtual void getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a, arr& B, arr& Bt, arr& Q, uint t);
  virtual void getControlCosts(arr& H, arr& Hinv, uint t);
  virtual void getTaskCosts(arr& phi, arr& phiJ, uint t);

  virtual void displayCurrentState(const char* title=NULL, bool pause=false, bool reportOnTasks=false);
  virtual void getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names);
};


#endif
