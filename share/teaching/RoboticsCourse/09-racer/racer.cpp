#include "racer.h"
#include <Algo/algos.h>
#include <Core/geo.h>

void drawEnv(void*);

Racer::Racer(){
  q.resize(2).setZero();
  q_dot.resize(2).setZero();
  q(1) = .01; //1MT_PI/2; //slighly non-upright //MT_PI; //haning down

  //init constants
  tau = 0.01;
  r = 0.05;  //radius of wheel
  l = 0.325; //height of pendulum COM
  lC = 0.49; //height of IMU
  mA = 0.05; //mass of wheel
  mB = 1.5;  //mass of pendulum
  IA = mA*MT::sqr(.9*r); //the mass averages at .9*r from the center
  IB = mB*MT::sqr(.8*l); //0.3705
  g = 9.81;
  noise_dynamics = 0.;

  c1 = 1./9.6; //9.81;
  c2 = 0.16-MT_PI/2.;
  c3 = 1.;
  c4 = -0.0417273;
  c5 = 1.;

  noise_accel = 1.;
  noise_gyro = .1;
  noise_enc = 1e-3;

  gl.add(drawEnv, this);
  gl.add(Racer::glStaticDraw, this);
  gl.camera.setPosition(0., -20., 5.);
  gl.camera.focus(0, 0, .2);
  gl.camera.upright();
  gl.update();
}

void Racer::getJacobians(arr& J_A, arr& J_B, arr& J_B_dash, arr& J_B_ddash, bool getJ_C){
  double L=l;
  if(getJ_C) L=lC;
  if(&J_A){
    J_A.resize(3,2).setZero();
    J_A(0,0)=1.;
    J_A(2,0)=1./r;
  }
  if(&J_B){
    J_B.resize(3,2).setZero();
    J_B(0,0) = J_B(2,1) = 1.;
    J_B(0,1) = +L*::cos(q(1));
    J_B(1,1) = -L*::sin(q(1));
  }
  if(&J_B_dash){
    J_B_dash.resize(3,2).setZero();
    J_B_dash(0,1) = -L*::sin(q(1));
    J_B_dash(1,1) = -L*::cos(q(1));
  }
  if(&J_B_ddash){
    J_B_ddash.resize(3,2).setZero();
    J_B_ddash(0,1) = -L*::cos(q(1));
    J_B_ddash(1,1) = +L*::sin(q(1));
  }
}

void Racer::getDynamics(arr& M, arr& F, arr& B, arr& M_dash, arr& M_ddash, arr& F_dash){
  B = ARR(1./r, -1.); //control matrix

  arr M_A, M_B;
  M_A.setDiag(ARR(mA,mA,IA));
  M_B.setDiag(ARR(mB,mB,IB));

  arr J_A, J_B, J_B_dash, J_B_ddash;
  getJacobians(J_A, J_B, J_B_dash, (&F_dash?J_B_ddash:NoArr));

  M = ~J_A * M_A * J_A + ~J_B * M_B * J_B;

  if(&M_dash || &F)
    M_dash = ~J_B * M_B * J_B_dash + ~J_B_dash * M_B * J_B;

  if(&F)
    F = q_dot(1)*M_dash*q_dot
        - ( 0.5*(~q_dot*M_dash*q_dot).scalar() + g*mB*l*::sin(q(1)) )*ARR(0., 1.);

  if(&F_dash){
    M_ddash = ~J_B * M_B * J_B_ddash + 2.*(~J_B_dash * M_B * J_B_dash) + ~J_B_ddash * M_B * J_B;
    F_dash = q_dot(1)*M_ddash*q_dot - (0.5*(~q_dot*M_ddash*q_dot).scalar() + g*mB*l*::cos(q(1)))*ARR(0., 1.);
  }
}

VectorFunction& Racer::dynamicsFct(){
  static struct DynFct:VectorFunction{
    Racer& R;
    DynFct(Racer& _R):R(_R){}
    void fv(arr& y, arr& J, const arr& q__q_dot){
      R.q = q__q_dot[0];
      R.q_dot = q__q_dot[1];

      arr M_dash, M_ddash, F_dash;
      R.getDynamics(R.M, R.F, R.B, M_dash, (&J?M_ddash:NoArr), (&J?F_dash:NoArr));
      inverse_SymPosDef(R.Minv, R.M);

      R.q_ddot = R.Minv * (R.B*R.u - R.F);

      if(&y) y = R.q_ddot;

      if(&J){
        J.resize(2,4).setZero();
        J.setMatrixBlock(-R.Minv*(M_dash*R.q_ddot + F_dash), 0, 1);
        arr F_q_dot = R.q_dot(1)*M_dash+(M_dash*R.q_dot)*~ARR(0.,1.) - ARR(0,1)*(~R.q_dot*M_dash);
        J.setMatrixBlock(-R.Minv*F_q_dot, 0, 2);
      }
    }
  } f(*this);
  return f;
}

void Racer::getDynamicsAB(arr& A, arr& a, arr& barB){
  u=0.;
  arr q_ddot, J;
  arr x=cat(q, q_dot).reshape(2,2);
  dynamicsFct().fv(q_ddot, J, x);
  A.resize(4,4).setZero();
  A.setMatrixBlock(eye(2),0,2);
  A.setMatrixBlock(J,2,0);
  x.reshape(4);
  a = cat(q_dot, q_ddot) - A*x;
  barB.resize(4,1).setZero();
  barB.setMatrixBlock(Minv*B,2,0);
}

double Racer::getEnergy(){
  arr M, F, B, M_dash;
  getDynamics(M, F, B, M_dash, NoArr, NoArr);
  double T = 0.5 * (~q_dot * M * q_dot).scalar(); //kinetic energy
  double U = g * mB * l * ::cos(q(1)); //potential energy
  cout <<"energy = " <<T+U <<endl;
  return T+U;
}

void Racer::getObservation(arr& y, arr& C, arr& c, arr& W){
  double ct=cos(q(1)+c2), st=sin(q(1)+c2);
  arr R = ARR(ct,-st,st,ct).reshape(2,2);
  arr R_dash = ARR(-st,-ct,ct,-st).reshape(2,2);

  arr J_C,J_C_dash,J_C_ddash;
  getJacobians(NoArr, J_C, J_C_dash, J_C_ddash, true);
  //pick only relevant (x,y) directions:
  J_C.resizeCopy(2,2);  J_C_dash.resizeCopy(2,2);  J_C_ddash.resizeCopy(2,2);

  //we need the dynamics
  arr q_ddot, J;
  dynamicsFct().fv(q_ddot, (&C?J:NoArr), cat(q, q_dot).reshape(2,2));
  q_ddot.setZero();

  //3-dimensional observation
  arr acc = (q_dot(1) * J_C_dash * q_dot + J_C * q_ddot) + ARR(0,g);
  y = c1 * R * acc; //2D: accelerations
  y.append(c3 * (q_dot(1)+c4)); //1D: gyro
  y.append(c5 * (q(0)/r-q(1))); //1D: encoder

  if(&C){
    arr acc_dash = q_dot(1) * J_C_ddash * q_dot + J_C_dash * q_ddot;
    arr acc_d_qdot = q_dot(1) * J_C_dash + (J_C_dash*q_dot)*~ARR(0.,1.);

    C.resize(4,6).setZero();
    C.setMatrixBlock(c1 * (R_dash * acc + R*acc_dash), 0, 1); //w.r.t. th
    C.setMatrixBlock(c1*R * acc_d_qdot, 0, 2); //w.r.t. q_dot
    C.setMatrixBlock(c1*R * J_C, 0, 4); //w.r.t q_ddot
    C(2, 3) = c3; //gyro
    C(3, 0) = c5/r; //encoder
    C(3, 1) = -c5; //encoder

    //use linearization of dynamics to resolve gradient w.r.t. q_ddot
    C = C.sub(0,-1,0,3) + C.sub(0,-1,4,5)*J;

    if(&c){
      c = y - C * cat(q, q_dot);
    }
  }

  if(&W){
    W = diag(ARR(noise_accel, noise_accel, noise_gyro, noise_enc));
  }
}

VectorFunction& Racer::observationFct(){
  static struct ObsFct:VectorFunction{
    Racer& R;
    ObsFct(Racer& _R):R(_R){}
    void fv(arr& y, arr& C, const arr& q__q_dot){
      R.q = q__q_dot[0];
      R.q_dot = q__q_dot[1];
      R.getObservation(y, C, NoArr, NoArr);
    }
  } f(*this);
  return f;
}

void Racer::stepDynamics(double _u){
  u=_u;
  arr x;
  rk4_2ndOrder(x, cat(q,q_dot).reshape(2,2), dynamicsFct(), tau);
  q=x[0];
  q_dot=x[1];

  if(noise_dynamics) rndGauss(q_dot, ::sqrt(tau)*noise_dynamics, true);
}

void Racer::stepDynamicsAcc(double u_acc){
  arr M_dash;
  getDynamics(M, F, B, M_dash);
  Minv = inverse_SymPosDef(M);
  arr BMB = inverse_SymPosDef((~B*Minv*B).reshape(1,1));
  u = (BMB * (u_acc + ~B*Minv*F)).scalar();
  stepDynamics(u);
}

void Racer::glDraw(){
  double GLmatrix[16];
  ors::Transformation f;
  f.setZero();
  //wheels
  f.addRelativeTranslation(q(0), 0, r);
  f.addRelativeRotationRad(q(0)/r, 0, 1, 0);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8,.2,.2);
  glDrawBox(2.*r, .01, 2.*r);
  //pole
  f.setZero();
  f.addRelativeTranslation(q(0), 0, r);
  f.addRelativeRotationRad(q(1), 0., 1., 0.);
  f.addRelativeTranslation(0., 0., l);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.2,.2,.2);
  glDrawBox(.01, .05, 2.*l);
  //sensor
  f.addRelativeTranslation(0., 0., lC-l);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.1,.2,.2);
  glDrawBox(.02, .05, .03);
  glLoadIdentity();
}

void Racer::glStaticDraw(void *classP){
  ((Racer*)classP)->glDraw();
}
