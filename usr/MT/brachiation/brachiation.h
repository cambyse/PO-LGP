#include <MT/octave.h>
#include <MT/socNew.h>

struct BrachiationSystem:ControlledSystem {
  arr x0,xT,QT,u,x;
  octave_value model;
  
  BrachiationSystem();

  uint get_T(){ return 61; }
  double get_tau(){ return 9.9300e-03; }
  uint get_xDim(){ return 8; }
  uint get_uDim(){ return 2; }
  uint get_phiDim(uint t){ if(t<get_T()) return 1; else return get_xDim(); }
  void get_x0(arr& x0){ x0 = this->x0; }
  bool isKinematic(){ return false; }
  void setx(const arr& x);
  arr& getx(){ return x; }

  void getDynamics(arr& A, arr& At, arr& Ainv, arr& Ainvt, arr& a,
		   arr& B, arr& Bt, arr& Q, uint t);
  void getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t);
  void getControlCosts(arr& H, arr& Hinv, uint t);
  void getTaskCosts(arr& phi, arr& phiJ, uint t);

  void displayCurrentState(const char* title=NULL, bool pause=false, bool reportOnTasks=false);
  void getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t);
};

