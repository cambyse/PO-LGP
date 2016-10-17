#include <Ors/ors.h>
#include <Optim/benchmarks.h>
#include <Motion/motion.h>
#include <Optim/optimization.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Control/taskController.h>
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
    cout << g << endl;

    // reduce Jg to only active part (lambda !=0)
    mlr::Array<uint> idx;
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
        ((RowShifted*)JgP.aux)->rowShift.remove(j);
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
      df.reshapeFlat();
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
  mlr::Array<Demonstration*> &demos;
  arr xOpt;
  uint numParam;
  uint numLambda;
  arr Dwdx;
  uint n;
  uint T;

  virtual uint dim_x() { return numParam;}
  virtual uint dim_g() { return numParam+numLambda+1;}

  IOC(mlr::Array<Demonstration*> &_demos,uint _numParam,bool _useDetH, bool _useHNorm):demos(_demos),numParam(_numParam) {
    n = demos(0)->MP.world.getJointStateDimension();
    T = demos(0)->MP.T;

    // precompute DWdx
    for (uint t=0;t<= T;t++) {
      // add transition cost elements
      Dwdx.append(catCol(eye(n),zeros(n,numParam-n)));

      // add task cost elements
      for (uint c=0;c<demos(0)->MP.tasks.N;c++) {
        if ( demos(0)->MP.tasks(c)->prec.N >t && (demos(0)->MP.tasks(c)->prec(t) > 0) && demos(0)->MP.tasks(c)->active && !demos(0)->MP.tasks(c)->map.constraint) {
          uint m = demos(0)->MP.tasks(c)->target.N;
          arr tmp = zeros(m,n);
          tmp = catCol(tmp,zeros(m,c));
          tmp = catCol(tmp,ones(m,1));
          tmp = catCol(tmp,zeros(m,numParam-tmp.d1));
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
      g.append((sumOfSqr(x)-1)*(sumOfSqr(x)-1)); // ||w|| = 1
      g.append(-x); // w > 0
    }
    if (&Jg) {
      Jg.clear();
      Jg=4.*~x*(sumOfSqr(x)-1);
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
  mlr::Array<Demonstration*> demos;

  // define toy demonstration 1
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);
  MotionProblem MP(world,true);
  MP.loadTransitionParameters();
  MP.makeContactsAttractive=false;
  arr refGoal1 = conv_vec2arr(MP.world.getBodyByName("goal1")->X.pos);
  arr refGoal2 = conv_vec2arr(MP.world.getBodyByName("goal2")->X.pos);
  TaskCost *c;
  c = MP.addTask("position_right_hand_1",new TaskMap_Default(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  c->setCostSpecs(200,200,refGoal1,1e4);
  c = MP.addTask("position_right_hand_2",new TaskMap_Default(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  c->setCostSpecs(120,120,refGoal2,1e3);
  c = MP.addTask("collisionConstraints", new PairCollisionConstraint(MP.world,"endeff","table",0.1));
  c->setCostSpecs(0, MP.T, {0.}, 1.);
  MP.x0 = {0.,0.,0.};
  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  optConstrained(x,lambda,Convert(MPF),OPT(verbose=0,stopTolerance=1e-7, maxStep=1.));
  cout << "lambda: "<< lambda << endl;
  MP.costReport(true);
  displayTrajectory(x,T,world,"optTraj");
  arr wGT;
  wGT.append(MP.H_rate_diag);
  wGT.append(MP.tasks(0)->prec(200));
  wGT.append(MP.tasks(1)->prec(120));



  // create MP for learning
  ors::KinematicWorld world2("scene");
  world2.getJointState(q, qdot);
  MotionProblem MP2(world2,true);
  MP2.loadTransitionParameters();
  MP2.makeContactsAttractive=false;
  TaskCost *c1;

  arr idx = linspace(0,T,10); idx.reshapeFlat();
  cout <<idx << endl;
  for (uint i =1;i<idx.d0;i++) {
    c1 = MP2.addTask("position_right_hand_1",new TaskMap_Default(posTMT,world2,"endeff", ors::Vector(0., 0., 0.)));
    c1->setCostSpecs(idx(i),idx(i),refGoal1,1.);
  }

  TaskCost *c2;
  for (uint i =1;i<idx.d0;i++) {
    c2 = MP2.addTask("position_right_hand_2",new TaskMap_Default(posTMT,world2,"endeff", ors::Vector(0., 0., 0.)));
    c2->setCostSpecs(idx(i),idx(i),refGoal2,1.);
  }

  TaskCost *c3 = MP2.addTask("collisionConstraints", new PairCollisionConstraint(MP2.world,"endeff","table",0.1));
  MP2.setInterpolatingCosts(c3, MotionProblem::constant, {0.}, 1.);

  MP2.x0 = {0.,0.,0.};

//  MotionProblemFunction MPF2(MP2);

  // save optimal solution for evaluation
  MP2.H_rate_diag = MP2.H_rate_diag/MP2.H_rate_diag;
  cout << MP2.H_rate_diag << endl;
  Demonstration* d = new Demonstration(MP2);
  d->x = x;
  d->lambda = lambda;
  demos.append(d);



  uint numParam = 2.*(idx.d0-1)+3;
  IOC ioc(demos,numParam,false,false);

  arr w = ones(numParam,1);w.reshapeFlat();
//  w=wOpt/sqrt(sumOfSqr(wOpt));
  //w = fabs(randn(numParam,1)); w.reshapeFlat();

//  checkAllGradients(ioc,w,1e-3);

  arr dual;
  optConstrained(w,dual,ioc,OPT(verbose=0,stopTolerance=1e-4,constrainedMethod=augmentedLag));
  w = fabs(w);
  w = w/sqrt(sumOfSqr(w))*1e3;
  cout << "GT w: "<< wGT/sqrt(sumOfSqr(wGT))<< endl;
  cout << "OPT w: "<< w/sqrt(sumOfSqr(w))<< endl;
  ioc.printOptSolution();

  // evaluation
  MP2.H_rate_diag(0) = w(0);
  MP2.H_rate_diag(1) = w(1);
  MP2.H_rate_diag(2) = w(2);
  for (uint i = 3;i<w.d0;i++) {
    MP2.tasks(i-3)->prec *= w(i);
  }

  arr x2(T+1,n); x2.setZero();arr lambda2(T+1); lambda2.setZero();
  MotionProblemFunction MPF2(MP2);

  optConstrained(x2,lambda2,Convert(MPF2),OPT(verbose=0,stopIters=500,stopTolerance=1e-4, maxStep=1.));
  MP2.costReport(true);
  cout << lambda2 << endl;

  displayTrajectory(x2,T,world2,"optTraj");
  MP2.world.watch(true);
}


int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);
  simpleMotion();

  return 0;
}


