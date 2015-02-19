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
    cout << _MP.world.getBodyByName("box")->X.pos << endl;
    cout << g << endl;

    MT::timerStart();
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

    Jg_JgtP = comp_A_At(JgP);
    Jg_Jgt = unpack(Jg_JgtP); Jg_Jgt.special = arr::noneST;
    Jgt_JgJgtI_Jg = ~Jg*inverse_SymPosDef(Jg_Jgt)*Jg;
    PHI = v.y;
    JP = v.Jy;

    arr J_JtP = comp_A_At(JP);
//    J_Jt = unpack(J_JtP); J_Jt.special = arr::noneST;
    J = unpack(JP); J.special = arr::noneST;
    dPHI_J_Jt_dPHI = (PHI*~PHI)%unpack(J_JtP);
//    dPHI_J_Jt_dPHI = diag(PHI)*unpack(J_JtP)*diag(PHI);
    cout << MT::timerRead(true) << endl;


//    arr Jgt_JgJgtI_JgP = ~JgP*inverse_SymPosDef(Jg_Jgt)*JgP;
//    cout << Jgt_JgJgtI_Jg << endl;

//    dWdx_dPHI_J_G_Jt_dPHI_dWdx = dPHI_J_Jt_dPHI - (diag(PHI)*J*Jgt_JgJgtI_Jg*~J*diag(PHI));
    dWdx_dPHI_J_G_Jt_dPHI_dWdx = dPHI_J_Jt_dPHI - ( (PHI*~PHI)%(J*Jgt_JgJgtI_Jg*~J));
//    dWdx_dPHI_J_G_Jt_dPHI_dWdx = dPHI_J_Jt_dPHI - ( (PHI*~PHI)%(JP*Jgt_JgJgtI_JgP*~JP));
    cout << MT::timerRead(true) << endl;

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
      for (uint c=0;c<demos(0)->MP.taskCosts.N;c++) {
        if ( demos(0)->MP.taskCosts(c)->prec.N >t && (demos(0)->MP.taskCosts(c)->prec(t) > 0) && demos(0)->MP.taskCosts(c)->active && !demos(0)->MP.taskCosts(c)->map.constraint) {

          uint m;
          if ((demos(0)->MP.taskCosts(c)->target.N-1) == T) {
            m = demos(0)->MP.taskCosts(c)->target.d1;
          } else {
            m = demos(0)->MP.taskCosts(c)->target.N;
          }

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
  MT::Array<Demonstration*> demos;

  // define toy demonstration 1 with object movement
  ors::KinematicWorld world("scene");
  for_list(ors::Joint, j, world.joints) {
    j->locked = false;
  }
  world.getBodyByName("box")->stiction = 0.;
  world.getBodyByName("box")->friction = 0.2;
  world.getBodyByName("box")->restitution = .1;
  world.getBodyByName("table")->stiction = 0.;
  world.getBodyByName("table")->friction = 0.2;
  world.getBodyByName("table")->restitution = .5;
  world.getBodyByName("endeff")->stiction = .2;
  world.getBodyByName("endeff")->friction = .1;
  world.getBodyByName("endeff")->restitution = .5;


  arr q, qdot;
  world.physx();
  world.getJointState(q, qdot);
  MotionProblem MP(world,false);
  MP.loadTransitionParameters();
  MP.makeContactsAttractive=false;
  arr refGoal1 = ARRAY(MP.world.getBodyByName("box")->X.pos)+ARRAY(0.,0.3,0.);
  arr refGoal2 = ARRAY(MP.world.getBodyByName("box")->X.pos)+ARRAY(0.,-0.2,0.);
  cout << refGoal1 << refGoal2 << endl;
  TaskCost *c;
  c = MP.addTask("position_right_hand_1",new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  c->setCostSpecs(200,200,refGoal1,1e5);
  c = MP.addTask("orientation_right_hand_1",new DefaultTaskMap(vecTMT,world,"endeff", ors::Vector(0., 0., 1.)));
  c->setCostSpecs(130,130,ARR(0.,1.,0.),1e1);
  c = MP.addTask("position_right_hand_2",new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
  c->setCostSpecs(140,140,refGoal2,1e4);

  c = MP.addTask("qItselfTMT", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, zeros(MP.world.getJointStateDimension(),1), 1e3);
  c->map.order=1;
  c = MP.addTask("collisionConstraints", new PairCollisionConstraint(MP.world,"endeff","table",0.0));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1.);
  MP.x0 = {0.,0.,0.,0.,0.};
  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  optConstrained(x,lambda,Convert(MPF),OPT(verbose=0,stopTolerance=1e-3, maxStep=1.));
  optConstrained(x,lambda,Convert(MPF),OPT(verbose=0,stopTolerance=1e-3, maxStep=1.));
  cout << "lambda: "<< lambda << endl;
  MP.costReport(true);


  arr boxTraj;
  arr lambdaTraj;
  arr xPosTraj;
  arr xVecTraj;
  for(uint t=0; t<=T; t++) {
    world.setJointState(x[t]);
    arr tmp;
    world.kinematicsPos(tmp,NoArr,world.getBodyByName("endeff"));
    xPosTraj.append(~tmp);
    //    ors::Vector a(0.,0.,1.);
    world.kinematicsVec(tmp,NoArr,world.getBodyByName("endeff"));
    xVecTraj.append(~tmp);
    world.physx().step(MP.tau);
    boxTraj.append(~ARRAY(world.getBodyByName("box")->X.pos));
    //    world.gl().update();
    lambdaTraj.append(~ARR(boxTraj(t,1)>1e-5));
    //    world.watch(false);
  }
  cout << boxTraj << endl;
  cout << lambdaTraj << endl;
  cout << x << endl;
  cout << xPosTraj << endl;


  // create MP for learning
  ors::KinematicWorld world2("scene");
  for_list(ors::Joint, j2, world2.joints) {
    j2->locked = false;
  }
  world2.getBodyByName("box")->stiction = 0.;  world2.getBodyByName("box")->friction = 0.2;  world2.getBodyByName("box")->restitution = .1;
  world2.getBodyByName("table")->stiction = 0.;  world2.getBodyByName("table")->friction = 0.2;  world2.getBodyByName("table")->restitution = .5;
  world2.getBodyByName("endeff")->stiction = .2;  world2.getBodyByName("endeff")->friction = .1;  world2.getBodyByName("endeff")->restitution = .5;

  world2.physx();
  world2.getJointState(q, qdot);
  MotionProblem MP2(world2,true);
  MP2.loadTransitionParameters();
  MP2.makeContactsAttractive=false;
  TaskCost *c1 = MP2.addTask("position_right_hand_1",new DefaultTaskMap(posTMT,world2,"endeff", ors::Vector(0., 0., 0.)));
  c1->setCostSpecs(138,138,xPosTraj[138],1.);

  TaskCost *c2 = MP2.addTask("position_right_hand_2",new DefaultTaskMap(posTMT,world2,"endeff", ors::Vector(0., 0., 0.)));
  c2->setCostSpecs(200,200,xPosTraj[200],1.);

  TaskCost *c6 = MP2.addTask("vec_right_hand_1",new DefaultTaskMap(vecTMT,world2,"endeff", ors::Vector(0., 0., 1.)));
  c6->setCostSpecs(138,138,xVecTraj[138],1.);


  TaskCost *c7 = MP2.addTask("vec_right_hand_2",new DefaultTaskMap(vecTMT,world2,"endeff", ors::Vector(0., 0., 1.)));
  c7->setCostSpecs(200,200,xVecTraj[200],1.);

//  TaskMap *tm_contact = new PairCollisionConstraint(MP2.world,"box","endeff",0.01);
//  TaskCost *c4 = MP2.addTask("contact_endeff",tm_contact);
//  c4->map.constraint = false;
//  //  c4->setCostSpecs(150,200,ARR(0.),1.);
//  c4->prec = lambdaTraj; c4->prec.flatten();
//  c4->target = lambdaTraj*0.;

  TaskCost *c5 = MP2.addTask("collisionConstraints", new PairCollisionConstraint(MP2.world,"box","endeff",0.0));
  MP2.setInterpolatingCosts(c5, MotionProblem::constant, ARRAY(0.), 1.);


  MP2.x0 = {0.,0.,0.,0.,0.};

  //  MotionProblemFunction MPF2(MP2);

  // save optimal solution for evaluation
  MP2.H_rate_diag = MP2.H_rate_diag/MP2.H_rate_diag;
  MP2.boxTraj = boxTraj;
  Demonstration* d = new Demonstration(MP2);
  d->x = x;
  d->lambda = lambdaTraj; d->lambda.flatten();
  demos.append(d);


  uint numParam = 5+4;
  IOC ioc(demos,numParam,false,false);

  arr w = ones(numParam,1);w.flatten();
  //  w=wOpt/sqrt(sumOfSqr(wOpt));
  //w = fabs(randn(numParam,1)); w.flatten();

    checkAllGradients(ioc,w,1e-3);
    return;

  arr dual;
  optConstrained(w,dual,ioc,OPT(verbose=1,stopIters=500,stopEvals=500,constrainedMethod=squaredPenalty));
  w = fabs(w);
  w = w/sqrt(sumOfSqr(w))*1e3;
  ioc.printOptSolution();

  // evaluation
  for (uint i = 0;i<MP2.H_rate_diag.d0;i++) {
    MP2.H_rate_diag(i) = w(i);
  }
  for (uint i = MP2.H_rate_diag.d0;i<w.d0;i++) {
    MP2.taskCosts(i-MP2.H_rate_diag.d0)->prec *= w(i);
  }

  arr x2(T+1,n); x2.setZero();arr lambda2(T+1); lambda2.setZero();
  MotionProblemFunction MPF2(MP2);

  optConstrained(x2,lambda2,Convert(MPF2),OPT(verbose=1,stopIters=500,stopEvals=500,stopTolerance=1e-5, maxStep=1.));
  MP2.costReport(true);
  cout << lambda2 << endl;

  while (true) {
    for(uint t=0; t<=T; t++) {
      world2.setJointState(x2[t]);
      world2.physx().step(MP2.tau);
      world2.gl().update();
    }
    world2.gl().watch();
  }

}


int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);
  simpleMotion();

  return 0;
}


