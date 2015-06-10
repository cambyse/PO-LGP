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

struct IOC_DemoCost {
  arr x0; // Demonstrated joint space
  arr lambda0; // Demonstrated constrains
  arr lambda; // Current lambda estimate
  arr Dwdx; // Matrix mapping x to w
  uint numParam; // number of x parameter
  bool useDetH;
  bool useHNorm;

  // task vars
  arr J_Jt,PHI,J,JP;
  // constrain vars
  arr Jg,g, JgP,Jg_Jgt,Jg_JgtP;
  arr J_Jgt;
  arr dWdx_dPHI_J_G_Jt_dPHI_dWdx;
  arr Jgt_JgJgtI_Jg;
  arr dPHI_J_Jt_dPHI;
  arr JgJgtI_Jg_J_dPHI;

  IOC_DemoCost(MotionProblem &_MP,arr &_x0,arr &_lambda0, arr &_Dwdx,uint _numParam,bool _useDetH, bool _useHNorm):x0(_x0),lambda0(_lambda0),Dwdx(_Dwdx),numParam(_numParam),useDetH(_useDetH),useHNorm(_useHNorm) {
    // precompute some terms
    MotionProblemFunction MPF(_MP);
    ConstrainedProblem & v = Convert(MPF);
    v.fc(NoArr,NoArr,g,JgP,_x0);

    // reduce Jg to only active part (lambda !=0)
    MT::Array<uint> idx;
    lambda0.findValues(idx,0.);
    lambda0.removeAllValues(0.);
    Jg = unpack(JgP);
    arr Jgr;
    uint j = 0;
    for (uint i =0;i<Jg.d0;i++) {
      if(!idx.contains(i)) {
        Jgr.append(~Jg[i]);
      }
      if(idx.contains(i)) {
        JgP.delRows(j);
        ((RowShiftedPackedMatrix*)JgP.aux)->rowShift.remove(j);
        j--;
      }
      j++;
    }
    Jg = Jgr;


    Jg_Jgt = unpack(comp_A_At(JgP));
    Jg_JgtP = comp_A_At(JgP);
    Jgt_JgJgtI_Jg = ~Jg*inverse(Jg*~Jg)*Jg;
    PHI = v.y;
    JP = v.Jy;
    J_Jt = unpack(comp_A_At(JP));
    J = unpack(JP); J.special = arr::noneST;
    dPHI_J_Jt_dPHI = diag(PHI)*J*~J*diag(PHI);
    dWdx_dPHI_J_G_Jt_dPHI_dWdx = dPHI_J_Jt_dPHI - (diag(PHI)*J*Jgt_JgJgtI_Jg*~J*diag(PHI));
    dWdx_dPHI_J_G_Jt_dPHI_dWdx =  ~Dwdx*dWdx_dPHI_J_G_Jt_dPHI_dWdx*Dwdx;
    JgJgtI_Jg_J_dPHI = inverse_SymPosDef(Jg_Jgt)*Jg*~J*diag(PHI);
    J_Jgt = J*~Jg;
  }

  double eval(arr& df, arr& Hf,arr& gLambda, arr& JgLambda, const arr& x) {
    // compute w vector
    arr w;
    w = Dwdx*x;
    arr PHIw = PHI%w;
    arr JP_PHIw = comp_At_x(JP,PHIw);
    arr J_Jt_PHIw = comp_A_x(JP,JP_PHIw);
    arr JgJgtI_Jg_Jt_PHIw = lapack_Ainv_b_sym(Jg_JgtP,comp_A_x(JgP,JP_PHIw));
    arr J_G_Jt_PHIw = J_Jt_PHIw - comp_A_x(J_Jgt,JgJgtI_Jg_Jt_PHIw);
    arr Hdxdx,HdxdxInv,Jw;

    lambda = -JgJgtI_Jg_Jt_PHIw;

    if (useHNorm || useDetH) {
      Jw = J%(repmat(sqrt(w),1,J.d1));
      Hdxdx = 2.*~Jw*Jw;
      lapack_inverseSymPosDef(HdxdxInv,Hdxdx);
    }

    arr f;
    if (useHNorm) {
      f = 4.*(~PHIw)*J*HdxdxInv*(~J)*PHIw;
    } else {
      f = 4.*(~PHIw)*J_G_Jt_PHIw;
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
        h = 8.*(PHI%(J_G_Jt_PHIw));
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
        Hf = 8.*(dWdx_dPHI_J_G_Jt_dPHI_dWdx);
      }

      if (useDetH){
        arr K = 2.*J*HdxdxInv*~J;
        Hf = Hf - ~Dwdx*(-K%K)*Dwdx;
      }
    }
    if (&gLambda) {
      gLambda = 2.*JgJgtI_Jg_Jt_PHIw;
    }

    if (&JgLambda) {
      JgLambda = 2.*JgJgtI_Jg_J_dPHI ;
    }

    return y;
  }
};

struct Demonstration {
  MotionProblem& MP; // MP containing the world state,
  arr x;             // joint trajectory
  arr lambda;        // constraint trajectory
  IOC_DemoCost* cost;// cost function for this demonstrations

  Demonstration (MotionProblem &_MP):MP(_MP) {

  }
};

struct IOC:ConstrainedProblem {
  MT::Array<Demonstration*> &demos;
  arr xOpt;
  uint numParam;
  uint numLambda;
  arr Dwdx;
  uint n;
  uint T;

  virtual uint dim_x() { return numParam;}
  virtual uint dim_g() { return numParam+numLambda+1;}

  IOC(MT::Array<Demonstration*> &_demos,uint _numParam,bool _useDetH, bool _useHNorm):demos(_demos),numParam(_numParam) {
    n = demos(0)->MP.world.getJointStateDimension();
    T = demos(0)->MP.T;

    // precompute DWdx
    for (uint t=0;t<= T;t++) {
      // add transition cost elements
      Dwdx.append(catCol(eye(n),zeros(n,numParam-n)));

      // add task cost elements
      for (uint c=0;c<demos(0)->MP.taskCosts.d0;c++) {
        if ( (demos(0)->MP.taskCosts(c)->prec(t) > 0) && demos(0)->MP.taskCosts(c)->active && !demos(0)->MP.taskCosts(c)->map.constraint) {
          arr tmp = zeros(demos(0)->MP.taskCosts(c)->target.d1,n);
          tmp = catCol(tmp,zeros(demos(0)->MP.taskCosts(c)->target.d1,c));
          tmp = catCol(tmp,ones(demos(0)->MP.taskCosts(c)->target.d1,1));
          tmp = catCol(tmp,zeros(demos(0)->MP.taskCosts(c)->target.d1,numParam-tmp.d1));
          Dwdx.append(tmp);
        }
      }
    }
    cout << "Dwdx: " << Dwdx << endl;

    numLambda = 0;
    // initialize cost functions for demonstrations
    for (uint i=0;i<demos.d0;i++) {
      demos(i)->cost = new IOC_DemoCost(demos(i)->MP,demos(i)->x,demos(i)->lambda,Dwdx,numParam,_useDetH,_useHNorm);
      numLambda += demos(i)->cost->lambda0.d0;
    }
  }


  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
    xOpt = x;

    double y = 0.;
    if (&df) {
      df.resize(numParam);df.setZero();
    }
    if (&Hf) {
      Hf.resize(numParam,numParam);Hf.setZero();
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

    // iterate over demonstrations
    for (uint i=0;i<demos.d0;i++) {
      arr dfi,Hfi,gi,Jgi;
      y += demos(i)->cost->eval(dfi, Hfi, &g?gi:NoArr, &Jg?Jgi:NoArr, x);

      if (&df) {
        df = df + dfi;
      }

      if (&Hf) {
        Hf += Hfi;
      }

      if (&g) {
        g.append(gi);
      }

      if (&Jg) {
        Jg.append(Jgi*Dwdx);
      }
    }

    return y;
  }

  void printOptSolution() {
    cout << "\n \n The solution is " <<xOpt << endl;
    for (uint i=0;i<demos.d0;i++) {
      cout << "\nDemonstration " << i << endl;
      cout << "Costs: " << demos(i)->cost->eval(NoArr,NoArr,NoArr,NoArr,xOpt) << endl;
      cout << "Lambda: " << demos(i)->cost->lambda/sqrt(sumOfSqr(demos(i)->cost->lambda)) << endl;
      cout << "Lambda GT: " << demos(i)->cost->lambda0/sqrt(sumOfSqr(demos(i)->cost->lambda0)) << endl;
    }
  }
};



void simpleMotion(){
  MT::Array<Demonstration*> demos;

  // define toy demonstration 1
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);
  MotionProblem MP(world,true);
  MP.loadTransitionParameters();
  MP.makeContactsAttractive=false;

  arr refGoal = ARRAY(MP.world.getBodyByName("goal")->X.pos);
  TaskCost *c;
  c = MP.addTask("position_right_hand",new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, refGoal, 1e4);
  c = MP.addTask("collisionConstraints", new PairCollisionConstraint(MP.world,"endeff","obstacle",0.1));
  MP.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1.);

  MP.x0 = {0.,0.,0.};

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  optConstrained(x,lambda,Convert(MPF),OPT(verbose=0,stopTolerance=1e-7, maxStep=1.));
  cout << "lambda: "<< lambda << endl;

  //  displayTrajectory(x,T,world,"optTraj");
  // save optimal solution for evaluation
  arr wGT;
  wGT.append(MP.H_rate_diag);
  wGT.append(MP.taskCosts(0)->prec(T));

  MP.taskCosts(0)->prec(T) = 1;
  MP.H_rate_diag = MP.H_rate_diag/MP.H_rate_diag;
  Demonstration* d = new Demonstration(MP);
  d->x = x;
  d->lambda = lambda;
  demos.append(d);




  // define toy demonstration 2
  ors::KinematicWorld world2("scene");
  world2.getJointState(q, qdot);
  MotionProblem MP2(world2,true);
  MP2.loadTransitionParameters();

  MP2.makeContactsAttractive=false;
  MP2.world.getBodyByName("goal")->X.pos += ARR(0.,0.,0.1);
  arr refGoal2 = ARRAY(MP2.world.getBodyByName("goal")->X.pos);
  TaskCost *c2;
  c2 = MP2.addTask("position_right_hand",new DefaultTaskMap(posTMT,world2,"endeff", ors::Vector(0., 0., 0.)));
  MP2.setInterpolatingCosts(c2, MotionProblem::finalOnly, refGoal2, 1e4);
  c2 = MP2.addTask("collisionConstraints", new PairCollisionConstraint(MP2.world,"endeff","obstacle",0.1));
  MP2.setInterpolatingCosts(c2, MotionProblem::constant, {0.}, 1.);

  MP2.x0 = {0.,0.,0.};
  MotionProblemFunction MPF2(MP2);
  arr x2(T+1,n); x2.setZero();arr lambda2(T+1); lambda2.setZero();
  optConstrained(x2,lambda2,Convert(MPF2),OPT(verbose=0,stopTolerance=1e-8, maxStep=1.));
//  displayTrajectory(x2,T,world2,"optTraj");
  cout << "lambda2: "<< lambda2 << endl;

  MP2.taskCosts(0)->prec(T) = 1;
  MP2.H_rate_diag = MP2.H_rate_diag/MP2.H_rate_diag;

  Demonstration* d2 = new Demonstration(MP2);
  d2->x = x2;
  d2->lambda = lambda2;

  demos.append(d2);


  uint numParam = 4;
  IOC ioc(demos,numParam,false,false);

  arr w = ones(numParam,1);w.flatten();
//  w=wOpt/sqrt(sumOfSqr(wOpt));
  //w = fabs(randn(numParam,1)); w.flatten();

  checkAllGradients(ioc,w,1e-3);

  arr dual;
  optConstrained(w,dual,ioc,OPT(verbose=2,stopTolerance=1e-5,constrainedMethod=augmentedLag));
  cout << "Ground Truth theta: " << wGT/sqrt(sumOfSqr(wGT)) << endl;
  ioc.printOptSolution();
}


int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);
  simpleMotion();

  return 0;
}

