#include <Ors/ors.h>
#include <Optim/benchmarks.h>
#include <Motion/motion.h>
#include <Optim/optimization.h>
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


struct Dfdw:ConstrainedProblem {
  //  MotionProblem MPF;
  ScalarFunction& s;
  VectorFunction& v;
  arr x0;
  arr Dwdx;
  bool useDetH;
  bool useHNorm;

  arr J_Jt,J_PHI,PHI,J,Jp;
  arr dPHI_J_Jt_dPHI;

  virtual uint dim_x() { return 4;}
  virtual uint dim_g() { return 1;}

  Dfdw(ScalarFunction& _fs,VectorFunction& _fv,arr &_x0,bool _useDetH, bool _useHNorm):s(_fs),v(_fv),x0(_x0),useDetH(_useDetH),useHNorm(_useHNorm) {
    // precompute some terms
    v.fv(PHI,Jp,x0);
    J_Jt = unpack(comp_A_At(Jp));
    J = unpack(Jp); J.special = arr::noneST;
    dPHI_J_Jt_dPHI = diag(PHI)*J*~J*diag(PHI);
  }

  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
//    MT::timerStart(true);
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
    arr J_Jt_PHIw = comp_A_x(Jp,comp_At_x(Jp,PHIw));

    arr Hdxdx,HdxdxInv,Jw;

    if (useHNorm || useDetH) {
      Jw = J%(repmat(sqrt(w),1,J.d1));
      Hdxdx = 2.*~Jw*Jw;
      lapack_inverseSymPosDef(HdxdxInv,Hdxdx);
    }

    arr f;
    if (useHNorm) {
      f = 4.*(~PHIw)*J*HdxdxInv*(~J)*PHIw;
    } else {
      f = 4.*(~PHIw)*J_Jt_PHIw;
    }

    double y = f(0);

    if (useDetH){
      double detHdxdx = lapack_determinantSymPosDef(Hdxdx);
      y = y - log(detHdxdx);
    }

    if (&df) {

      arr h;
      if (useHNorm) {
        h = 8.*(PHI-(J*HdxdxInv*~J*PHIw)) % (J*HdxdxInv*~J*PHIw);
      } else {
        h = 8.*(PHI%(J_Jt_PHIw));
      }

      df = ~h*Dwdx ;

      if (useDetH) {
        arr g2 = getDiag(2.*(J*~HdxdxInv*(~J)));
        g2 = ~g2*Dwdx;
        df = df - g2;
      }
      df.flatten();
    }

    if (&Hf) {
      if (useHNorm) {
        Hf = -16.*diag(J*HdxdxInv*~J*PHIw)*(J*HdxdxInv*~J*(-2.*diag(J*HdxdxInv*~J*PHIw) + diag(PHI)));
        Hf = Hf + 8.*diag(PHI)*J*HdxdxInv*~J*(-2.*diag(J*HdxdxInv*~J*PHIw) + diag(PHI));
      } else {
        Hf = 8.*(dPHI_J_Jt_dPHI);
      }

      Hf = ~Dwdx*Hf*Dwdx ;

      if (useDetH){
        arr K = 2.*J*HdxdxInv*~J;
        Hf = Hf - ~Dwdx*(-K%K)*Dwdx;
      }
    }

    if (&g) {
      g.resize(dim_g());g.setZero();
      g(0) = (sumOfSqr(x)-1.)*(sumOfSqr(x)-1.);
    }

    if (&Jg) {
      Jg.resize(dim_g(),dim_x());Jg.setZero();
      Jg=4.*~x*(sumOfSqr(x)-1.);
    }
//    cout << "4: "  << MT::timerRead(true) << endl;

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

  Dfdw dfdw(f,fv,x,true,false);

  arr gi,Hi,PHIi,Ji;
  arr w = sqr(PHI/(PHIo+1e-12));
  //  checkGradient(dfdw,x,1e-3);
  w = w.subRange(w.d0-6,w.d0-3);
  //  w=ARRAY(1.,2.,3.,4.);
  w = randn(4,1)+2.;
  w.reshape(w.N);
  w = ARR(2.25127, 1.0321, 1.90551, 1.13059);

  checkAllGradients(dfdw,w,1e-3);
  cout << w/sqrt(sumOfSqr(w)) << endl;


  arr dual;
  optConstrained(w,dual,dfdw,OPT(verbose=2,stopTolerance=1e-6, maxStep=1.,stopIters = 5000,stopEvals=5000,constrainedMethod=squaredPenalty));
  cout << "Found Solution: " << w << endl;
  cout << "Opt Solution: " << wOpt/sqrt(sumOfSqr(wOpt)) << endl;
}


int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);
  //    gradCheckExample();
  simpleMotion();

  return 0;
}

