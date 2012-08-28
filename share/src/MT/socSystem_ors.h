#ifndef MT_socSystem_ors_h
#define MT_socSystem_ors_h

#include "soc.h"
#include "ors.h"

//===========================================================================
//
// ORS simulator implementation of the SocAbstration
//

namespace soc {

/** \brief an implementation of the SocSystemAbstraction using the \ref ors
    simulator */
struct SocSystem_Ors: public virtual SocSystemAbstraction {
  ors::Graph *ors;
  SwiftInterface *swift;
  MT::Array<TaskVariable*> vars;
  struct sSocSystem_Ors *s;
  
  SocSystem_Ors();
  virtual ~SocSystem_Ors();
  SocSystem_Ors* newClone(bool deep) const;
  
  //initialization methods
  void initBasics(ors::Graph *_ors, SwiftInterface *_swift, OpenGL *_gl,
                  uint trajectory_steps, double trajectory_time, bool _dynamic, arr *W);
  void setTimeInterval(double trajectory_time, uint trajectory_steps);
  void setTaskVariables(const TaskVariableList& CVlist);
  
  //--exemplary problem setups: read specifications from MT.cfg
  void initStandardReachProblem(uint rand_seed=0, uint T=0, bool _dynamic=false);
  void initStandardBenchmark(uint rand_seed=0);
  
  //info
  void reportOnState(std::ostream& os);
  void displayState(const arr *x, const arr *Q=NULL, const char *text=NULL, bool reportVariables=false);
  void recordTrajectory(const arr& q,const char *variable,const char *file);

  //implementations of virtual methods
  uint get_T();
  uint get_xDim();
  uint nTasks();
  uint qDim();
  uint uDim();
  uint yDim(uint i);
  void getq0(arr& q);
  void setq0(const arr& q0);
  void getv0(arr& v);
  void get_x0(arr& x);
  void setx0(const arr& x0);
  void getqv0(arr& q, arr& qd);
  bool isDynamic();
  void setq(const arr& q, uint t=0);
  void setx(const arr& x, uint t=0);
  void setqv(const arr& q, const arr& qd, uint t=0);
  void setx0ToCurrent();
  //void geth  (arr& h);
  virtual void getControlCosts(arr& H, arr& Hinv, uint t);
  void getHrateInv(arr& HrateInv);
  bool isConditioned(uint i, uint t);
  bool isConstrained(uint i, uint t);
  const char* taskName(uint i);
  uint taskDim(uint i);
  void getPhi(arr& phiq_i, uint i);
  void getJJt(arr& J_i, arr& Jt_i, uint i);
  void getHessian(arr& H_i, uint i);
  void getJqd(arr& Jqd_i, uint i);
  void getTarget(arr& y_i, double& prec, uint i, uint t);
  void getTargetV(arr& v_i, double& prec, uint i, uint t);
  //void getC   (arr& C_i, uint i, uint t);
  //void getCV  (arr& D_i, uint i, uint t);
  double getTau(bool scaled=true);
  void getMF(arr& M, arr& F, arr& Q, uint t);
  void getMinvF(arr& Minv, arr& F, arr& Q, uint t);
  virtual void setTau(double tau);
};

}

#endif
