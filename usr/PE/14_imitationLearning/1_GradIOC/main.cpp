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

  SquareFunction f;
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

  Dfdw(ScalarFunction& _fs,VectorFunction& _fv,arr &_x0):s(_fs),v(_fv),x0(_x0) {

  }
  virtual double fs(arr& g, arr& H, const arr& x) {
    //    double y = s.fs(g,H,x);
    arr PHI,J;
    v.fv(PHI,J,x0);
    J = unpack(J);

    // compute w vector
    arr w=repmat(cat(x.subRange(0,2),zeros(3,1)),x0.d0,1);
    w.subRange(w.d0-3,w.d0-1)=x(3);
//    cout << w << endl;
    // compute matrix dWdx
    arr Dwdx(w.d0,x.d0); Dwdx.setZero();
    arr tmp = ARRAY(1.,0.,0.,0.,0.,0.);
    tmp = repmat(tmp,x0.d0,1);
    Dwdx = ~tmp;
    tmp.shift(1,false);
    Dwdx.append(~tmp);
    tmp.shift(1,false);
    Dwdx.append(~tmp);
    Dwdx.append(~tmp*0.);
    Dwdx = ~Dwdx;
    Dwdx(Dwdx.d0-3,3)=Dwdx(Dwdx.d0-2,3)=Dwdx(Dwdx.d0-1,3)=1.;

    arr f1;
    for (uint i=0;i<x0.N;i++) {
      f1.append(~(J.col(i))*(PHI%w)*2.);
    }
    arr f2 = ~f1*f1;
    if (&g) {
      arr h(w.N);h.setZero();

//      arr g2;g2.resize(w.N);g2.setZero();
      for (uint i=0;i<x0.N;i++) {
        h = h + (4.*(f1(i))*(J.col(i)%PHI));
//        g2 = g2 + (4.*(~(J.col(i))*(PHI%w))*(J.col(i)%(PHI)));
      }
//      for (uint j=0;j<w.N;j++){
//        for (uint i=0;i<x0.N;i++) {
//          g(j) = g(j)+(4.*f1(i)*J(j,i)*PHI(j));
//        }
//      }
      g = ~h*Dwdx;
//      g2 = ~g2*Dwdx;
      cout << "g"<< g << endl;

    }

    if (&H) {
      arr K(w.N,w.N);K.setZero();
      H.resize(w.N,w.N);H.setZero();
      for (uint i=0;i<x0.N;i++) {
        K = K + (8.*J.col(i)%PHI)*~(J.col(i)%PHI);
      }

      H = ~Dwdx*K*Dwdx;
      cout << H << endl;
    }

    cout << "f2 "<<f2 << endl;
    return f2(0);
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
  c = MP.addTaskMap("position_right_hand",new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));

  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, refGoal, 25);
  MP.x0 = {0.,0.,0.};
  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();
  optNewton(x,Convert(MPF),OPT(verbose=0,stopTolerance=1e-4, maxStep=1.));
  //  displayTrajectory(x,T,world,"optTraj",0.1);

  arr g,H,PHI,J;
  ScalarFunction & f = Convert(MPF);
  VectorFunction & fv = Convert(MPF);

  f.fs(g,H,x);
  fv.fv(PHI,J,x);

  cout << "g: "<< g.prt() << endl;
  cout << "H: "<< H.prt() << endl;
  cout << "PHI: "<<PHI.prt() << endl;
  cout << "J: "<<J.prt() << endl;

  MP.taskCosts(0)->y_prec(T) = 1;
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
  w=ARRAY(1.,1.,1.,1.);

  cout << w << endl;

//  cout << dfdw.fs(gi,Hi,w+0.01) << endl;
//  cout << gi << endl;
  checkGradient(dfdw,w,1e-3);
  checkHessian(dfdw,w,1e-3);


  optNewton(w,dfdw,OPT(verbose=0,stopTolerance=1e-4, maxStep=1.));
}





int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  //  gradCheckExample();
  simpleMotion();





  /*
  arr g,H,PHI,J;
  ScalarFunction & f = Convert(MPF);
  VectorFunction & fv = Convert(MPF);

  f.fs(g,H,x);
  fv.fv(PHI,J,x);

  arr w = repmat(cat(MP.H_rate_diag,ARR(1e4,1e4,1e4)),T+1,1);
  arr W((T+1)*8,(T+1)*8);W.setZero();
  for (uint c = 0;c<W.d0;c++) {
    W(c,c) = w(c,0);
  }

  // compute matrices without weighting
  arr w2 = repmat(cat(MP.H_rate_diag,ARR(1e4,1e4,1e4)),T+1,(1+k)*n);
  arr PHIo = PHI%(1./sqrt(w));
  arr Jo = J%(1./sqrt(w2));

  arr costs = ~g*g;
  arr costs2 = 4.*(~PHI)*unpack(J)*(~unpack(J))*PHI;
  arr costs3 = 4.*(~PHIo)*W*unpack(Jo)*(~unpack(Jo))*W*PHIo;

  arr costsdW =8.*(Jo*(~Jo)*W*PHIo*(~PHIo));

  cout << costs <<" "<< costs2 <<" "<< costs3 << endl;


  arr Dwdw;//((T+1)*8*(T+1)*8,8);Dwdw.setZero();
  arr tmp; tmp = ARR(1.,0.,0.,0.,0.,0.,0.,0.);

  arr tmp2;
  for (uint c = 0;c<8;c++) {
    tmp2 = repmat(tmp,(T+1)*8*(T+1),1);
    Dwdw.append(~tmp2);
    tmp.shift(1,true);
  }
  Dwdw = ~Dwdw;

  costsdW = (~costsdW).flatten();
  
  
  cout << ~costsdW*Dwdw << endl;
  
  
  return 0;
  cout << "g: " << g.d0 << " x " << g.d1<< " "<< sum(g) << "\n"  << endl;
  cout << "H: " << H.d0 << " x " << H.d1 << "\n" <<H << endl;


  arr HK =  unpack(H);
  arr Evals,Evecs;
  lapack_EigenDecomp(HK,Evals,Evecs);
  cout << Evals << endl;
*/  return 0;
}

