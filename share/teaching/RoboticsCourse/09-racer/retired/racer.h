#include <Core/array.h>
#include <Gui/opengl.h>

struct Racer{
  //state
  arr q, q_dot;
  //buffer of results calling fv
  arr q_ddot,M,Minv,F,B;

  //controls
  double u, tau;

  //constant parameters
  double r, l, lC, mA, mB, IA, IB, g, noise_dynamics; //dynamics parameters
  double c1, c2, c3, c4, c5, noise_accel, noise_gyro, noise_enc; //observation parameters

  OpenGL gl;
  Racer();
  void getJacobians(arr& J_A, arr& J_B, arr& J_B_dash=NoArr, arr &J_B_ddash=NoArr, bool getJ_C=false);
  void getDynamics(arr& M, arr& F, arr& B, arr &M_dash=NoArr, arr &M_ddash=NoArr, arr &F_dash=NoArr);
  void getDynamicsAB(arr& A, arr& a, arr& barB);
  double getEnergy();
  void getObservation(arr& y, arr& C=NoArr, arr& c=NoArr, arr &W=NoArr);
  void stepDynamics(double _u);
  void stepDynamicsAcc(double u_acc);

  VectorFunction& dynamicsFct(); //returns the acceleration given the state -> used by rk4
  VectorFunction& observationFct();
  void glDraw();
  static void glStaticDraw(void*);


};
