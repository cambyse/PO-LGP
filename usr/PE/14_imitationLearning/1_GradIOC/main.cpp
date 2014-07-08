#include <Ors/ors.h>
#include <Optim/benchmarks.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <vector>
#include <future>
#include <GL/glu.h>
#include <Gui/opengl.h>
#include <Ors/ors_physx.h>

void gradCheckExample(){
  arr x = ARR(0.,1.);
  double tol = 1e-3;

  ScalarFunction& f = SquareFunction;

  checkGradient(f,x,tol);
  checkHessian(f,x,tol);

  SquaredCost f2(2);
  checkJacobian(f2,x,tol);
}


struct Dfdw:ScalarFunction {
  //  MotionProblem MPF;
  ScalarFunction& s;
  VectorFunction& v;
  arr x0;
  arr Dwdx;

  Dfdw(ScalarFunction& _fs,VectorFunction& _fv,arr &_x0):s(_fs),v(_fv),x0(_x0) {

  }
  virtual double fs(arr& g, arr& H, const arr& x) {
    //    double y = s.fs(g,H,x);
    arr PHI,J;
    v.fv(PHI,J,x0);
    J = unpack(J);
    J.special = arr::noneST;

    // compute w vector
    arr w=repmat(x.subRange(0,2),x0.d0,1.);
    w.append(repmat(ARR(x(3)),3,1));
    w.flatten();

    // compute matrix dWdx
    if (Dwdx.d0<1.) {
      arr tmp = ARRAY(1.,0.,0.);
      tmp = repmat(tmp,x0.d0,1);
      tmp.append(repmat(ARR(0.),3,1));
      Dwdx = ~tmp;
      tmp.shift(1,false);
      Dwdx.append(~tmp);
      tmp.shift(1,false);
      Dwdx.append(~tmp);
      Dwdx.append(~tmp*0.);
      Dwdx = ~Dwdx;
      Dwdx(Dwdx.d0-3,3)=Dwdx(Dwdx.d0-2,3)=Dwdx(Dwdx.d0-1,3)=1.;
    }

    arr PHIw = PHI%w;
    arr Jw = J%(repmat(sqrt(w),1,J.d1));
    arr f = 4.*(~PHIw)*J*(~J)*PHIw;

    arr Hdxdx = 2.*~Jw*Jw;
    double detHdxdx = lapack_determinantSymPosDef(Hdxdx);
    double y = f(0) - log(detHdxdx);

    arr HdxdxInv;
    if ((&g) || (&H)) {
      lapack_inverseSymPosDef(HdxdxInv,Hdxdx);
    }

    if (&g) {
      arr h = 8.*(PHI%(J*~J*PHIw));
      g = ~h*Dwdx ;

      arr g2 = getDiag(2.*(J*~HdxdxInv*(~J)));
      g2 = ~g2*Dwdx;
      g = g - g2;
      g.flatten();
    }

    if (&H) {
      H = 8.*(diag(PHI)*J*~J*diag(PHI));
      H = ~Dwdx*H*Dwdx ;

      arr K = 2.*J*HdxdxInv*~J;
      H = H - ~Dwdx*(-K%K)*Dwdx;
    }

    return y;
  }
};

void simpleMotion(){
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);

  MotionProblem MP(world);
  MP.useSwift = false;
  MP.loadTransitionParameters();

  arr refGoal = ARRAY(MP.world.getBodyByName("goal")->X.pos);
  TaskCost *c;

  c = MP.addTask("position_right_hand",new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, refGoal, 25);
  MP.x0 = {0.,0.,0.};
  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();
  optNewton(x,Convert(MPF),OPT(verbose=0,stopTolerance=1e-4, maxStep=1.));
  //    displayTrajectory(x,T,world,"optTraj",0.1);

  arr g,H,PHI,J;
  ScalarFunction & f = Convert(MPF);
  VectorFunction & fv = Convert(MPF);

  f.fs(g,H,x);
  fv.fv(PHI,J,x);

  cout << "g: "<< g << endl;
  cout << "H: "<< H << endl;
  cout << "PHI: "<<PHI << endl;
  cout << "J: "<<J << endl;
  cout << "H: "; H.write(cout);cout <<"\n";

  arr wOpt;
  wOpt.append(MP.H_rate_diag);
  wOpt.append(MP.taskCosts(0)->prec(T));

  MP.taskCosts(0)->prec(T) = 1;
  MP.H_rate_diag = MP.H_rate_diag/MP.H_rate_diag;

  arr go,Ho,PHIo,Jo;

  f.fs(go,Ho,x);
  fv.fv(PHIo,Jo,x);
  cout << PHIo << endl << PHI << endl;

  Dfdw dfdw(f,fv,x);

  arr gi,Hi,PHIi,Ji;
  arr w = sqr(PHI/(PHIo+1e-12));
  //  checkGradient(dfdw,x,1e-3);
  w = w.subRange(w.d0-6,w.d0-3);
  //  w=ARRAY(1.,2.,3.,4.);
  w = randn(4,1)+2.;
  w.reshape(w.N);
  cout << w << endl;
  w = ARR(2.25127, 1.0321, 1.90551, 1.13059);

  //  cout << dfdw.fs(gi,Hi,w+0.01) << endl;
  //  cout << gi << endl;
  checkGradient(dfdw,w,1e-3);
  checkHessian(dfdw,w,1e-3);

  optNewton(w,dfdw,OPT(verbose=0,stopTolerance=1e-3, maxStep=1.,stopIters = 20000,stopEvals=20000));

  cout << w/sqrt(sumOfSqr(w)) << endl;
  cout << wOpt/sqrt(sumOfSqr(wOpt)) << endl;
}





int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  //    gradCheckExample();
  simpleMotion();




  return 0;
}

