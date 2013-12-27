#include <Core/array.h>
#include <Gui/opengl.h>

struct Racer : VectorFunction{
  //state
  arr q, q_dot;
  //buffer of results calling fv
  arr q_ddot,M,Minv,F,B;

  //controls
  double u, tau;

  //constant parameters
  double r, l, mA, mB, IA, IB, g, noise_dynamics; //dynamics parameters
  double c1, c2, noise_accel, noise_gyro; //observation parameters

  OpenGL gl;
  Racer();
  void getJacobians(arr& J_A, arr& J_B, arr& J_B_dash, arr &J_B_ddash);
  void getDynamics(arr& M, arr& F, arr& B, arr &M_dash, arr &M_ddash, arr &F_dash);
  void getDynamicsAB(arr& A, arr& a, arr& barB);
  double getEnergy();
  void getObservation(arr& y, arr& C, arr& c, arr &W);
  void fv(arr& q_ddot, arr& J, const arr& q__q_dot); //returns the acceleration given the state -> used by rk4
  void stepDynamics(double _u);

  VectorFunction& getObs();
  void glDraw();
  static void glStaticDraw(void*);


};
