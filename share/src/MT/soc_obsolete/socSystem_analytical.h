#ifndef MT_socSystem_analytical_h
#define MT_socSystem_analytical_h

#include "soc.h"

//===========================================================================
//
// toy implementation of the SocAbstration
//

namespace soc {

/** \brief an implementation of the SocSystemAbstraction that simulates a
    single 1D point mass on a spring */
struct SocSystem_Analytical: public virtual SocSystemAbstraction{
  struct SocSystem_Analytical_Workspace *s;

  SocSystem_Analytical();
  virtual ~SocSystem_Analytical();

  void initKinematic(uint dim, uint trajectory_length, double w, double endPrec);
  void initDynamic(uint dim, double trajectory_time, uint trajectory_steps, arr *H=NULL);

  
  //implementations of virtual methods
  uint nTime();
  uint nTasks();
  uint qDim();
  uint uDim();
  uint yDim(uint i);
  void getq0(arr& q);
  void setq0(const arr& q);
  void getv0(arr& v);
  void getx0(arr& x);
  void getqv0(arr& q, arr& qd);
  bool isDynamic();
  void setq(const arr& q, uint t=0);
  void setx(const arr& x, uint t=0);
  void setqv(const arr& q, const arr& qd, uint t=0);
  void setx0ToCurrent();
  //void geth  (arr& h);
  void getW(arr& W, uint t);
  void getWinv(arr& Winv, uint t);
  void getH(arr& H, uint t);
  void getHinv(arr& Hinv, uint t);
  void getQ(arr& Q, uint t);
  void getHrateInv(arr& HrateInv);
  void getQrate(arr& Qrate);      
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
  void getMF(arr& M, arr& F, uint t);
  void getMinvF(arr& Minv, arr& F, uint t);
  virtual void setTau(double tau);
};

}

#endif