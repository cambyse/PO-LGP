#include "execution.h"
#include <Motion/taskMap_proxy.h>
#include <Ors/ors_swift.h>
#include <Core/geo.h>
#include <vector>


using namespace std;


void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, arr x0, const double& height, bool stickyness, uint horizon){
    /////////////

  //SET INITIAL STATE: Model parameter (height, position, size,...), and initial joint's position
  //world.setJointState(x0);
  //world.getBodyByName("table")->X.pos.z = height;


  /////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  MotionProblem P(world, true); //true for using swift
  P.loadTransitionParameters(); // can change horizon hereP

  P.T = horizon;
  x = P.getInitialization();

  //-- setup the motion problem


  TaskCost *c;

  TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "peg", NoVector, "target", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 2e5);

  TaskCost *vel = P.addTask("position_vel", new DefaultTaskMap(posTMT, world, "peg", NoVector));
  vel->map.order=1;
  P.setInterpolatingCosts(vel, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  //see taskmap_default.cpp;
  TaskCost *vec = P.addTask("orientation", new DefaultTaskMap(vecTMT, world, "peg",ARRAY(0.,0.,1.)));
  //P.setInterpolatingCosts(vec, MotionProblem::finalOnly, ARRAY(0.,0.,-1.), 1e3, ARRAY(0.,0.,0.), 1e-3);
  P.setInterpolatingCosts(vec, MotionProblem::early_restConst, ARRAY(0.,0.,-1.), 1e3, NoArr, -1., 0.1);

  /*/ors::Quaternion target;
  target.setRadZ(3.14);
  target.setRadY(3.14/2.0); target, positionPrec, NoArr, -1., .8);
  target.setRadX(-3.14/2.0);

  cout<<target<<endl;

  TaskCost *vec = P.addTask("orientation", new DefaultTaskMap(quatTMT, world, "peg"));
  P.setInterpolatingCosts(vec, MotionProblem::finalOnly, ARRAY(target.w,target.x,target.y,target.z), 1e3);/*/


  TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "peg", ARR(0,0,-1, 0.2 + 0.05)));//0.2 is table width  //0.05 above table surface to avoid slippery
  P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1e2);


#if 1  //CONSTRAINT
  TaskCost *collision = P.addTask("collisionConstraint", new CollisionConstraint());
  P.setInterpolatingCosts(collision, MotionProblem::constant, ARRAY(0.), 1e3);
#else
  c = P.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .041));
  P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e1);
#endif


  if(stickyness){

    TaskCost *sticky = P.addTask("planeStickiness", new ConstraintStickiness(cons->map));
    sticky->setCostSpecs(0, P.T, {0.}, 1.);

    P.makeContactsAttractive = true;
  }else{
    //stickyWeight = 0.;
    P.makeContactsAttractive = false;
  }


  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;



  for(uint k=0;k<20;k++){
   optNewton(x, UnConstrainedP, OPT(verbose=0, stopIters=300, damping=1e-4, stopTolerance=1e-5, maxStep=.5));
    P.costReport();
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    UnConstrainedP.aulaUpdate(.9,x);
    P.dualMatrix = UnConstrainedP.lambda;
    UnConstrainedP.mu *= 2.;
 }

  //get the final optimal cost at each time slice
  P.costReport();

  if(&y){
    y.resize(x.d0, pos->map.dim_phi(world));

    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }
  uint index = 0;
  dual.resize(x.d0);
  if(&dual) {
      for(int i=1;i<UnConstrainedP.lambda.d0;i=i+2){
          dual(index) = UnConstrainedP.lambda(i);

          index++;
      }
  }

}


// DUAL-EXECUTION
void POMDPExecution(const arr& x, const arr& y, const arr& dual, ors::KinematicWorld& world, int num){
  arr q, qdot;
  world.getJointState(q, qdot);

  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("peg");
  ors::Shape *true_target = world.getShapeByName("target");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("hole");
  double mean_table_height = table->X.pos.z;

  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

  FeedbackMotionControl MC(world);
  MC.qitselfPD.active=false;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8
  PDtask *pd_y =  MC.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "peg", NoVector, "target"));
  pd_y->prec = 10.;


  //joint space PD task
  PDtask *pd_x = MC.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "peg", ARR(0,0,-1, table->X.pos.z + 0.2 + 0.05));  //0.05 above table surface to avoid slippery
  ConstraintForceTask *pd_c = MC.addConstraintForceTask("planeConstraint", plane_constraint );

  MC.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});

#endif

  double tau = 0.01;

  // remaining 100 steps is for reaching to the target.
  for(uint t=0;t<x.d0 + 100;t++){
    MC.setState(q, qdot);

   //adapt the PD task references following the plan

   //cout<< pd_y->y_ref <<endl;
   //cout<<"endeff->X.pos "<< endeff->X.pos <<endl;

   if(t<y.d0){
        pd_y->y_ref = y[t];
        pd_x->y_ref = x[t];
  #ifdef USE_DUAL       
        pd_c->desiredForce = dual(t);
  #endif
    }

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z - 0.2; //est_target position update
      }
    }
#endif


 //   //external sinus on the table height
 //   table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
#ifdef USE_DUAL
//    plane_constraint->planeParams(3) = table->X.pos.z + 0.2; //0.2 + 0.02
#endif

    //operational space loop
    for(uint tt=0;tt<10;tt++){
      MC.updateConstraintControllers();
      arr a = MC.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    //display and record video
//    world.watch(false, STRING(t));
    world.gl().update(STRING(t), true, false, true);
    //    flip_image(world.gl().captureImage);
    //    vid->addFrame(world.gl().captureImage);

    //write data
    MT::arrayBrackets="  ";
    data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
        <<table->X.pos.z <<' '
       <<endeff->X.pos.z <<' '
      <<endeff->X.pos.z-table->X.pos.z <<' '
      <<est_target->X.pos.z <<' '
     <<true_target->X.pos.z <<' '
    <<endl;
  }
  data.close();

  FILE(STRING("data-"<<num<<"-err.dat")) << ARRAY(true_target->X.pos)- ARRAY(endeff->X.pos);
}

