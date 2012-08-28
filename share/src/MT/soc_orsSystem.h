#ifndef MT_socSystem_ors_h
#define MT_socSystem_ors_h

#include "socNew.h"
#include "ors.h"

//===========================================================================
//
// ORS simulator implementation of the SocAbstration
//

/** \brief an implementation of the SocSystemAbstraction using the \ref ors
    simulator */
struct OrsSystem: ControlledSystem {
  struct sOrsSystem *s;
  
  OrsSystem();
  virtual ~OrsSystem();
  OrsSystem* newClone(bool deep) const;
  
  //-- initialization methods
  void initBasics(ors::Graph *_ors, SwiftInterface *_swift, OpenGL *_gl,
                  uint trajectory_steps, double trajectory_time, bool _dynamic, arr *W);
  //-- exemplary problem setups: read specifications from MT.cfg
  void initStandardReachProblem(uint rand_seed=0, uint T=0, bool _dynamic=false);
  void initStandardBenchmark(uint rand_seed=0);

  //-- junk
  void setTau(double tau);
  void setTimeInterval(double trajectory_time, uint trajectory_steps);
  void setTaskVariables(const TaskVariableList& CVlist);
  uint get_qDim();
  ors::Graph& getOrs();
  
  //-- implementations of virtual methods
  uint get_T();
  double get_tau();
  uint get_xDim();
  uint get_uDim();
  uint get_phiDim(uint t);
  void get_x0(arr& x0);
  bool isKinematic();
  void setx(const arr& x);
  arr& getx();
  void getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t);
  void getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a, arr& B, arr& Bt, arr& Q, uint t);
  void getControlCosts(arr& H, arr& Hinv, uint t);
  void getTaskCosts(arr& phi, arr& phiJ, uint t);
  void displayCurrentState(const char* title=NULL, bool pause=false, bool reportOnTasks=false);
  void getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t);
};

#endif
