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



struct IOC_DemoCost {
  arr x0;
  arr Dwdx;
  uint numParam;
  bool useDetH;
  bool useHNorm;

  arr J_Jt,PHI,J,Jp;
  arr dPHI_J_Jt_dPHI;


  IOC_DemoCost(MotionProblem &_MP,arr &_x0,arr &_Dwdx,uint _numParam,bool _useDetH, bool _useHNorm):x0(_x0),Dwdx(_Dwdx),numParam(_numParam),useDetH(_useDetH),useHNorm(_useHNorm) {
    // precompute some terms
    MotionProblemFunction MPF(_MP);
    VectorFunction & v = Convert(MPF);
    v.fv(PHI,Jp,x0);
    J_Jt = unpack(comp_A_At(Jp));
    J = unpack(Jp); J.special = arr::noneST;
    dPHI_J_Jt_dPHI = diag(PHI)*J*~J*diag(PHI);
  }

  double eval(arr& df, arr& Hf, const arr& x) {
    //    MT::timerStart(true);
    // compute w vector
    arr w;

    w = Dwdx*x;
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

    //    cout << "4: "  << MT::timerRead(true) << endl;
    return y;
  }
};

struct Demonstration {
  MotionProblem& MP; // MP containing the world state,
  arr x;             // joint trajectory
  IOC_DemoCost* cost;// cost function for this demonstrations

  Demonstration (MotionProblem &_MP):MP(_MP) {

  }
};

struct IOC:ConstrainedProblem {
  uint numParam;
  arr Dwdx;
  uint n;
  uint T;

  MT::Array<Demonstration*> &demos;

  virtual uint dim_x() { return numParam;}
  virtual uint dim_g() { return numParam+1;}

  IOC(MT::Array<Demonstration*> &_demos,uint _numParam,bool _useDetH, bool _useHNorm):demos(_demos),numParam(_numParam) {
    n = demos(0)->MP.world.getJointStateDimension();
    T = demos(0)->MP.T;

    // precompute DWdx
    for (uint t=0;t<= T;t++) {
      // add transition cost elements
      Dwdx.append(catCol(eye(n),zeros(n,numParam-n)));

      // add task cost elements
      for (uint c=0;c<demos(0)->MP.taskCosts.d0;c++) {
        if ( (demos(0)->MP.taskCosts(c)->prec(t) > 0) && demos(0)->MP.taskCosts(c)->active) {
          arr tmp = zeros(demos(0)->MP.taskCosts(c)->target.d1,n);
          tmp = catCol(tmp,zeros(demos(0)->MP.taskCosts(c)->target.d1,c));
          tmp = catCol(tmp,ones(demos(0)->MP.taskCosts(c)->target.d1,1));
          tmp = catCol(tmp,zeros(demos(0)->MP.taskCosts(c)->target.d1,numParam-tmp.d1));
          Dwdx.append(tmp);
        }
      }
    }
    cout << "Dwdx: " << Dwdx << endl;

    // initialize cost functions for demonstrations
    for (uint i=0;i<demos.d0;i++) {
      demos(i)->cost = new IOC_DemoCost(demos(i)->MP,demos(i)->x,Dwdx,numParam,_useDetH,_useHNorm);
    }
  }


  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {

    double y = 0.;
    if (&df) {
    df.resize(numParam);df.setZero();
    }
    if (&Hf) {
      Hf.resize(numParam,numParam);Hf.setZero();
    }

    // iterate over demonstrations
    for (uint i=0;i<demos.d0;i++) {
      arr dfi,Hfi;
      y += demos(i)->cost->eval(dfi,Hfi,x);

      if (&df) {
        df = df + dfi;
      }

      if (&Hf) {
        Hf += Hfi;
      }
    }

    if (&g) {
      g.clear();
      g.append((sumOfSqr(x)-1.)*(sumOfSqr(x)-1.)); // ||w|| = 1
      g.append(-x); // w > 0
    }

    if (&Jg) {
      Jg.clear();
      Jg=4.*~x*(sumOfSqr(x)-1.);
      Jg.append(-eye(numParam));
    }
    return y;
  }
};



void simpleMotion(){
  MT::Array<Demonstration*> demos;

  // define toy demonstration 1
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

  c = MP.addTask("vec_right_hand", new DefaultTaskMap(vecAlignTMT,world,"endeff", ors::Vector(0., 0., 1.),"goal",ors::Vector(0.,0.,-1.)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARR(1.), 25);
  c = MP.addTask("qItselfTMT", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, zeros(MP.world.getJointStateDimension(),1), 1);
  c->map.order=1;
  MP.x0 = {0.,0.,0.};
  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();
  optNewton(x,Convert(MPF),OPT(verbose=0,stopTolerance=1e-4, maxStep=1.));
//  displayTrajectory(x,T,world,"optTraj");

  // save optimal solution for evaluation
  arr wOpt;
  wOpt.append(MP.H_rate_diag);
  wOpt.append(MP.taskCosts(0)->prec(T));
  wOpt.append(MP.taskCosts(1)->prec(T));
  wOpt.append(MP.taskCosts(2)->prec(T));


  MP.taskCosts(0)->prec(T) = 1;
  MP.taskCosts(1)->prec(T) = 1;
  MP.taskCosts(2)->prec(T) = 1;
  MP.H_rate_diag = MP.H_rate_diag/MP.H_rate_diag;

  Demonstration* d = new Demonstration(MP);
  d->x = x;
  demos.append(d);





  // define toy demonstration 2
  ors::KinematicWorld world2("scene");
  world.getJointState(q, qdot);
  MotionProblem MP2(world2);
  MP2.useSwift = false;
  MP2.loadTransitionParameters();
  MP2.world.getBodyByName("goal")->X.pos += ARR(0.,0.2,0.);
  arr refGoal2 = ARRAY(MP2.world.getBodyByName("goal")->X.pos);
  TaskCost *c2;
  c2 = MP2.addTask("position_right_hand",new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  MP2.setInterpolatingCosts(c2, MotionProblem::finalOnly, refGoal2, 20);
  c2 = MP2.addTask("vec_right_hand", new DefaultTaskMap(vecAlignTMT,world,"endeff", ors::Vector(0., 0., 1.),"goal",ors::Vector(0.,0.,-1.)));
  MP2.setInterpolatingCosts(c2, MotionProblem::finalOnly, ARR(1.), 25);
  c2 = MP2.addTask("qItselfTMT", new DefaultTaskMap(qItselfTMT,world));
  MP2.setInterpolatingCosts(c2, MotionProblem::finalOnly, zeros(MP2.world.getJointStateDimension(),1), 1);
  c2->map.order=1;
  MP2.x0 = {0.,0.,0.};
  MotionProblemFunction MPF2(MP2);
  arr x2(T+1,n); x.setZero();
  optNewton(x2,Convert(MPF2),OPT(verbose=0,stopTolerance=1e-4, maxStep=1.));
//  displayTrajectory(x2,T,world2,"optTraj");

  MP2.taskCosts(0)->prec(T) = 1;
  MP2.taskCosts(1)->prec(T) = 1;
  MP2.taskCosts(2)->prec(T) = 1;
  MP2.H_rate_diag = MP2.H_rate_diag/MP2.H_rate_diag;

  Demonstration* d2 = new Demonstration(MP2);
  d2->x = x2;
  demos.append(d2);

  uint numParam = 6;
  IOC ioc(demos,numParam,false,false);


  arr w = fabs(randn(numParam,1)); w.flatten();

//  checkAllGradients(ioc,w,1e-3);
  //  //  return;
  arr dual;
  optConstrained(w,dual,ioc,OPT(verbose=2,stopTolerance=1e-5,constrainedMethod=augmentedLag));
  cout << "Found Solution: " << w << endl;
  cout << "Opt Solution: " << wOpt/sqrt(sumOfSqr(wOpt)) << endl;
}


int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);
  //  gradCheckExample();
  simpleMotion();

  return 0;
}


