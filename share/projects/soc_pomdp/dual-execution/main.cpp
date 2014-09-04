#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

extern double stickyWeight;

//VideoEncoder_libav_simple *vid;

void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, const double& height, arr& values){
  //set the sampled model's paramter
  world.getBodyByName("table")->X.pos.z = height;

  MotionProblem P(world, false);
  P.loadTransitionParameters();
  x = P.getInitialization();

  //-- setup the motion problem
  TaskCost *pos =
      P.addTask("position",
                   new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,
                          ARRAY(0.,0.,0.), 1e3);
//                          ARRAY(P.world.getShapeByName("target")->X.pos), 1e3);
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1,.7)));
    P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1.);

 //P.addTask("collisionConstraints", new CollisionConstraint());
  //TaskCost *coll = P.addTask("collisionConstraints", new CollisionConstraint());

  //P.setInterpolatingCosts(coll, MotionProblem::constant, ARRAY(0.), 1.);

  stickyWeight=1.;
  P.makeContactsAttractive = true;

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, UnConstrainedP, OPT(verbose=0, stopIters=100, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
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
  if(&dual) dual = UnConstrainedP.lambda;
}



/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void POMDPExecution(const arr& allx, const arr& ally, const arr& alldual, ors::KinematicWorld& world, int num){
  arr q, qdot;
  world.getJointState(q, qdot);

  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("endeff");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");
  double mean_table_height = table->X.pos.z;

  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

  FeedbackMotionControl MC(world);
  MC.qitselfPD.active=false;

  //position PD task
  PDtask *pd_y=
      MC.addPDTask("position", .1, .8,
                   new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x=
      MC.addPDTask("pose", .1, .8,
                    new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeff", ARR(0,0,-1,table->X.pos.z+0.02));
  ConstraintForceTask *pd_c =
      MC.addConstraintForceTask("planeConstraint", plane_constraint );
//      MC.addConstraintForceTask("touchTable",
//                                new PairCollisionConstraint(world, "endeff2", "table"));
#endif


  double tau = 0.01;
  arr x = allx[0];
  arr y = ally[0];
  arr dual = alldual[0];

  //loop over time

  // remaining 100 steps is for reaching to the target.
  for(uint t=0;t<x.d0+100;t++){
    MC.setState(q, qdot);

    //adapt the PD task references following the plan
    if(t<y.d0){
      pd_y->y_ref = y[t];
      pd_x->y_ref = x[t];
#ifdef USE_DUAL
      pd_c->desiredForce=dual(t);
#endif
    }
    //observation: equivalent to touch or not?
#ifdef USE_DUAL

#endif

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z+0.1; //est_target position update
      }
    }
#endif
    //external sinus on the table height
    table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
#ifdef USE_DUAL
    plane_constraint->planeParams(3) = table->X.pos.z + 0.02;
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


////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
///////////////////////////////////////////////////////////////////////////////
int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));
  uint T = 200;

  MT::timerStart(true);

  //compute the primal and dual trajectories
  arr heights;
  uint numSamples = 10;
  heights.resize(numSamples);
  arr allX, allY, allDual;
  arr values; //2-dim: sample, time
  values.resize(numSamples,T+1);

  for(uint i=0;i<numSamples;i++){
      //1. very large variance (1.0)
      heights(i) = .6 + 0.1*rnd.gauss();
      //2. trajectory optimization: return primal,dual trajectories, and value functions (at each time slice)
      arr x, y, dual;
      getTrajectory(x, y, dual, world, heights(i),values[i]());

      allX.resize(numSamples,x.d0,x.d1);
      allY.resize(numSamples,y.d0,y.d1);
      allDual.resize(numSamples,dual.d0);

      //3. store the trajectories
      allX[i]() = x;
      allY[i]() = y;
      allDual[i]() = dual;
  }

  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;

  //TESTING: Online POMDP planning
  //POMDP
  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;
  world.setJointState(allX[0][0]);
  for(uint i=0;i<10;i++){
    world.getBodyByName("table")->X.pos.z = .6 + 0.1*rnd.gauss();
    POMDPExecution(allX, allY, allDual, world, i);
  }

  return 0;
}


