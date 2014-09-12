#include <pr2/actionMachine.h>
#include <pr2/actions.h>

#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

//extern double stickyWeight;

//VideoEncoder_libav_simple *vid;

void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world, const double& height, uint horizon){
    /////////////


  //set the sampled model's paramter
  cout<<world.getBodyByName("target")->X.pos.z<<endl;// = height;

  //cout<< "height= "<<world.getBodyByName("table")->X.pos.z<<endl;

  //world.setJointState(ARR(0, -1, -1, 2, -1, 0, 0));


  MotionProblem P(world, false);
  P.loadTransitionParameters(); // can change horizon here


  P.T = horizon;

  x = P.getInitialization();

  world.getBodyByName("target")->X.pos.z = height + 0.12;
  TaskCost *pos = P.addTask("position", new DefaultTaskMap(posTMT, world, "endeffR", NoVector, "target", NoVector));
  P.setInterpolatingCosts(pos, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 1e3);


  TaskCost *cons = P.addTask("planeConstraint", new PlaneConstraint(world, "endeffR", ARR(0,0,-1, height+0.02)));
  P.setInterpolatingCosts(cons, MotionProblem::constant, ARRAY(0.), 1e4);

  TaskCost *collision = P.addTask("collisionConstraint", new CollisionConstraint());
  P.setInterpolatingCosts(collision, MotionProblem::constant, ARRAY(0.), 1e3);


  TaskCost *sticky = P.addTask("planeStickiness", new ConstraintStickiness(cons->map));
  sticky->setCostSpecs(0, P.T, {0.}, 1.);

  P.makeContactsAttractive = true;

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);

  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, UnConstrainedP, OPT(verbose=0, stopIters=100, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
    P.costReport(false);
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    UnConstrainedP.aulaUpdate(.9,x);

    P.dualMatrix = UnConstrainedP.lambda;
    UnConstrainedP.mu *= 2.;

  }
  //get the final optimal cost at each time slice
  P.costReport(false);

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
void PR2_POMDPExecution(ActionSystem& activity, const arr& x, const arr& y, const arr& dual, int num){

  ors::KinematicWorld world = activity.machine->s->world;

  arr q, qdot;
  world.getJointState(q, qdot);

  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("endeffR");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");
  double mean_table_height = table->X.pos.z;

  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

  //FeedbackMotionControl MC(world);
  //MC.qitselfPD.active=true;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8
  PDtask *pd_y =  activity.machine->s->MP.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeffR", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x = activity.machine->s->MP.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;

  //plane constraint task
#define USE_DUAL
#ifdef USE_DUAL
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1,table->X.pos.z+0.02));
  ConstraintForceTask *pd_c =
      activity.machine->s->MP.addConstraintForceTask("planeConstraint", plane_constraint );
//      MC.addConstraintForceTask("touchTable",
//                                new PairCollisionConstraint(world, "endeff2", "table"));
#endif


  double tau = 0.01;


  // remaining 100 steps is for reaching to the target.
  for(uint t=0;t<x.d0 + 100;t++){
    activity.machine->s->MP.setState(q, qdot);

    cout<< q<<endl;

      if(t<y.d0){
        pd_y->y_ref = y[t];
        pd_x->y_ref = x[t];
  #ifdef USE_DUAL
        pd_c->desiredForce = dual(t);

        //cout<< x[t] <<endl;
  #endif
    }

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){
      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z + 0.1; //est_target position update
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
      activity.machine->s->MP.updateConstraintControllers();
      arr a = activity.machine->s->MP.operationalSpaceControl();
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

// ============================================================================
int main(int argc, char** argv)
{

  MT::initCmdLine(argc, argv);
  //ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));
  ActionSystem activity;
  //activity.machine->add(new CoreTasks());
 /*/ PDtask* limits = activity.machine->s->MP.addPDTask("limits", .1, .8, qLimitsTMT);
  // limits->setGains(10.,0.);
  limits->v_ref.setZero();
  limits->v_ref.setZero();
  limits->prec=100.;
  //tasks.append(limits);

  PDtask* coll = activity.machine->s->MP.addPDTask(
      "collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
    coll->y_ref.setZero();
    coll->v_ref.setZero();

/*/



  arr q,qdot;
  activity.machine->s->world.getJointState(q,qdot);
  double table_height = activity.machine->s->world.getBodyByName("table")->X.pos.z;
  cout<< "planning's table_height " <<table_height<<endl;

  uint T = 200; //time horizon

  MT::timerStart(true);

  //compute the primal and dual trajectories
  arr heights;
  uint numSamples = 3;
  heights.resize(numSamples);


  double height = table_height + 0.1*rnd.gauss();
  //2. trajectory optimization: return primal,dual trajectories, and value functions (at each time slice)

  arr x, y, dual;
  getTrajectory(x, y, dual, activity.machine->s->world, height, T);

  cout<<dual<<endl;

  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;


  //TESTING: Online POMDP planning
  //POMDP

  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

  for(uint i=0;i<1;i++){
    activity.machine->s->world.setJointState(q);
    activity.machine->s->world.getBodyByName("table")->X.pos.z = table_height + 0.1*rnd.gauss();
    PR2_POMDPExecution(activity, x, y, dual, i);
  }

  return 0;
}
