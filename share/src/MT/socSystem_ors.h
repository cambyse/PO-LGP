#ifndef MT_socSystem_ors_h
#define MT_socSystem_ors_h

#include "soc.h"

//===========================================================================
// @}
// ORS simulator implementation of the SocAbstration
//

namespace soc {

struct SocSystem_Ors_Workspace;

/** \brief an implementation of the SocSystemAbstraction using the \ref ors
    simulator */
struct SocSystem_Ors: public virtual SocSystemAbstraction {
  ors::Graph *ors;
  SwiftInterface *swift;
  MT::Array<TaskVariable*> vars;
  SocSystem_Ors_Workspace *WS;
  
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
  void displayState(const arr *x, const arr *Q, const char *text=NULL, bool reportVariables=false);
  
  //implementations of virtual methods
  uint nTime();
  uint nTasks();
  uint qDim();
  uint uDim();
  uint yDim(uint i);
  void getq0(arr& q);
  void setq0(const arr& q);
  void getv0(arr& v);
  void getqv0(arr& q_);
  void getqv0(arr& q, arr& qd);
  bool isDynamic();
  void setq(const arr& q, uint t=0);
  void setqv(const arr& q_, uint t=0);
  void setqv(const arr& q, const arr& qd, uint t=0);
  void setq0AsCurrent();
  //void geth  (arr& h);
  void getW(arr& W, uint t);
  void getWinv(arr& Winv, uint t);
  void getH(arr& H, uint t);
  void getHinv(arr& Hinv, uint t);
  void getTotalHinv(arr& Hinv); //new
  void getQ(arr& Q, uint t);
  void getTotalQ(arr& Q);//new
  bool isConditioned(uint i, uint t);
  bool isConstrained(uint i, uint t);
  const char* taskName(uint i);
  void getPhi(arr& phiq_i, uint i);
  void getJJt(arr& J_i, arr& Jt_i, uint i);
  void getHessian(arr& H_i, uint i);
  void getJqd(arr& Jqd_i, uint i);
  void getTarget(arr& y_i, double& prec, uint i, uint t);
  void getTargetV(arr& v_i, double& prec, uint i, uint t);
  //void getC   (arr& C_i, uint i, uint t);
  //void getCV  (arr& D_i, uint i, uint t);
  double getTau(bool scaled=true);
  void getMF(arr& M, arr& F, uint t);
  void getMinvF(arr& Minv, arr& F, uint t);
  
};

}

#endif
