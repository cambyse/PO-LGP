#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <pr2/actions.h>
#include <pr2/actionMachine.h>

#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>

#include <Optim/optimization.h>
#include <Core/util.h>
//#include <Perception/videoEncoder.h>


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
void POMDPExecution(ors::KinematicWorld& world, const arr& x, const arr& y, const arr& dual, int num){


    arr q, qdot;
    world.getJointState(q, qdot);

    ofstream data(STRING("data-"<<num<<".dat"));

    ors::Shape *endeff = world.getShapeByName("endeffR");
    ors::Shape *true_target = world.getShapeByName("truetarget");
    ors::Body *est_target = world.getBodyByName("target");
    ors::Body *table = world.getBodyByName("table");


    double mean_table_height = table->X.pos.z;

    double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);

    FeedbackMotionControl MP(world);
    MP.qitselfPD.active=true;

    //position PD task:  decayTime = 0.1, dampingRatio = 0.8

    est_target->X.pos.z = table->X.pos.z + 0.12;

    cout<< "est_target->X.pos.z  = "<<est_target->X.pos.z<<endl;
    cout<< "true_target->X.pos.z  = "<<true_target->X.pos.z<<endl;


    PDtask *pd_y =  MP.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeffR", NoVector, "target"));
    pd_y->prec = 10.;

    //joint space PD task
    PDtask *pd_x = MP.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
    pd_x->prec = .1;


    //PDtask* coll = activity.machine->s->MP.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
    //  coll->y_ref.setZero();
    //  coll->v_ref.setZero();


    //plane constraint task
  #define USE_DUAL
  #ifdef USE_DUAL
    PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1,table->X.pos.z+0.02));
    ConstraintForceTask *pd_c =
        MP.addConstraintForceTask("planeConstraint", plane_constraint );
  //      MC.addConstraintForceTask("touchTable",
  //                                new PairCollisionConstraint(world, "endeff2", "table"));
  #endif


    double tau = 0.01;

    cout<<"target "<<est_target->X.pos.z <<endl;

    // remaining 100 steps is for reaching to the target.
    for(uint t=0;t<x.d0 + 100;t++){
      MP.setState(q, qdot);

     // cout<< q<<endl;

        if(t<y.d0){
          pd_y->y_ref = y[t];
          pd_x->y_ref = x[t];
    #ifdef USE_DUAL
          pd_c->desiredForce = dual(t);
    #endif
      }

        if(t%10==0)   cout<<"  endeff "<<endeff->X.pos.z<<endl;

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
       MP.updateConstraintControllers();
        arr a =MP.operationalSpaceControl();
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


/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void PR2_POMDPExecution(ActionSystem& activity, const arr& x, const arr& y, const arr& dual, int num){

  ors::KinematicWorld& world = activity.machine->s->world;

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
  activity.machine->s->MP.qitselfPD.active=true;

  //position PD task:  decayTime = 0.1, dampingRatio = 0.8

  est_target->X.pos.z = table->X.pos.z + 0.12;

  cout<< "est_target->X.pos.z  = "<<est_target->X.pos.z<<endl;
  cout<< "true_target->X.pos.z  = "<<true_target->X.pos.z<<endl;


  PDtask *pd_y =  activity.machine->s->MP.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeffR", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x = activity.machine->s->MP.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;


  //PDtask* coll = activity.machine->s->MP.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
  //  coll->y_ref.setZero();
  //  coll->v_ref.setZero();


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

  cout<<"target "<<est_target->X.pos.z <<endl;

  // remaining 100 steps is for reaching to the target.
  for(uint t=0;t<x.d0 + 100;t++){
    activity.machine->s->MP.setState(q, qdot);

   // cout<< q<<endl;

      if(t<y.d0){
        pd_y->y_ref = y[t];
        pd_x->y_ref = x[t];
  #ifdef USE_DUAL
        pd_c->desiredForce = dual(t);
  #endif
    }

      cout<<"  endeff "<<endeff->X.pos.z<<endl;

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
    //table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
#ifdef USE_DUAL
    //plane_constraint->planeParams(3) = table->X.pos.z + 0.02;
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





///////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////
struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, gamepadState);
  ACCESS(arr, wrenchL)
  ACCESS(arr, wrenchR)
  MySystem(){
    addModule<GamepadInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
      addModule<RosCom_ForceSensorSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
    connect();
  }
};

/// Online execution: Using POMDP policy (solve the POMDP online, using offline value functions from SOC)
void PR2_ActionMachine(ors::KinematicWorld& world, const arr& x, const arr& y, const arr& dual, int num){

 // ors::KinematicWorld& world = activity.machine->s->world;
  MySystem S;
  engine().open(S);
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  ors::Joint *trans=world.getJointByName("worldTranslationRotation");
  ors::Shape *ftL_shape=world.getShapeByName("endeffL");

  ors::KinematicWorld worldCopy = world;

  //world.gl().add(ors::glDrawGraph, &worldCopy);

  FeedbackMotionControl MP(world, true); // true means using swift
  //MP.qitselfPD.y_ref = q;
  MP.H_rate_diag = pr2_reasonable_W(world);

  bool useRos = MT::getParameter<bool>("useRos", false);
  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      S.ctrl_obs.var->waitForNextRevision();
      if(S.ctrl_obs.get()->q.N==MP.world.q.N
         && S.ctrl_obs.get()->qdot.N==MP.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S.ctrl_obs.get()->q;
    qdot = S.ctrl_obs.get()->qdot;
    //arr fL_base = S.fL_obs.get();
    MP.setState(q, qdot);
  }

  arr zero_qdot(qdot.N);
  zero_qdot.setZero();
  CtrlMsg refs;



  /////////////////////////////////////////////////////////////////////////////////
  // FROM old code
  ////////////////////////////////////////////////////////////////////////////////
  MP.qitselfPD.active=true;


  ors::Shape *endeff = world.getShapeByName("endeffR");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");


  double sin_jitter = MT::getParameter<double>("sin_jitter", 0.);



  PDtask *pd_y =  MP.addPDTask("position", .1, .8, new DefaultTaskMap(posTMT, world, "endeffR", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  PDtask *pd_x = MP.addPDTask("pose", .1, .8, new DefaultTaskMap(qItselfTMT, world));
  pd_x->prec = .1;


/*/

  PDtask* limits = MP.addPDTask("limits", .1, .8, qLimitsTMT);
  // limits->setGains(10.,0.);
  limits->v_ref.setZero();
  limits->v_ref.setZero();
  limits->prec=100.;
  //tasks.append(limits);

  PDtask* coll = MP.addPDTask("collisions", .2, .8, collTMT, NULL, NoVector, NULL, NoVector, {.1});
  coll->y_ref.setZero();
  coll->v_ref.setZero();/*/

  cout<<"table->X.pos.z "<<table->X.pos.z<<"  "<< endeff->X.pos.z<< endl;

  //PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1,table->X.pos.z+0.02));
  PlaneConstraint *plane_constraint = new PlaneConstraint(world, "endeffR", ARR(0,0,-1,0.2));
  ConstraintForceTask *pd_c = MP.addConstraintForceTask("planeConstraint", plane_constraint );


  ///////////////////////////////////////////

bool updated =false;
  for(uint t=0;t<x.d0 + 100;t++){
      MT::wait(.1);

      MP.setState(q, qdot);
    ////////////////////////////////////////////////////////////////////////////////
      if(t<y.d0){
        pd_y->y_ref = y[t];
        pd_x->y_ref = x[t];
        pd_c->desiredForce = dual(t);
    }
    ////////////////////////////////////////////////////////////////////////////////


    // joint state
    if(useRos){
      worldCopy.setJointState(S.ctrl_obs.get()->q, S.ctrl_obs.get()->qdot);
    }

    //cout<< endeff->X.pos.z<< endl;

////////////////////////////////////////////////////////////////////////////////
    //recalibrate the target based on touch
    double d=0.;
    arr f_r = S.wrenchR.get();// ctrl_obs.get()->fR;

    cout<< f_r<<endl;

    if((!updated)&&(f_r(1)<4.0)){
        est_target->X.pos.z = endeff->X.pos.z + 0.1;
        plane_constraint->planeParams(3) = endeff->X.pos.z;

        cout<<"updated"<<endl;
        updated = true;
    }


    if(pd_c->desiredApproach.y.N){

      //cout<<

      d = pd_c->desiredApproach.y(0); //d = distance measured by constraint task
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){
        est_target->X.pos.z = endeff->X.pos.z + 0.1; //est_target position update


        // UPDATE table's height and plane constraint when first touch with the table
       // plane_constraint->planeParams(3) = endeff->X.pos.z;
       // table->X.pos.z = endeff->X.pos.z - 0.02;

       //  cout<<"table->X.pos.z "<<table->X.pos.z<<"  "<< endeff->X.pos.z<< endl;
      }
    }
////////////////////////////////////////////////////////////////////////////////

    //compute control

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .001*qdot;
      qdot += .001*a;
    }


 // MP.reportCurrentState();

    //MP.world.reportProxies();
    //if(!(t%4))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    refs.q = q;
    refs.qdot = zeros(q.N);
    refs.u_bias = zeros(q.N);

    S.ctrl_ref.set() = refs;


  }

  engine().close(S);
  cout <<"bye bye" <<endl;


}

// ============================================================================
int main(int argc, char** argv)
{

  MT::initCmdLine(argc, argv);

#if 0
  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));
  //ActionSystem activity;
  //activity.machine->add(new CoreTasks());



  arr q,qdot;
  world.getJointState(q,qdot);
  double table_height = world.getBodyByName("table")->X.pos.z;
  cout<< "planning's table_height " <<table_height<<endl;

  uint T = 200; //time horizon

  MT::timerStart(true);

  //compute the primal and dual trajectories
  arr heights;
  uint numSamples = 3;
  heights.resize(numSamples);


  double height = table_height;// + 0.1*rnd.gauss();
  //2. trajectory optimization: return primal,dual trajectories, and value functions (at each time slice)

  arr x, y, dual;
  getTrajectory(x, y, dual, world, height, T);

  for(int t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      world.gl().update(STRING(t), true, false, true);
  }

  cout<<dual<<endl;

  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;


  //TESTING: Online POMDP planning
  //POMDP

  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

  for(uint i=0;i<5;i++){
    world.setJointState(q);
    world.getBodyByName("table")->X.pos.z = table_height;// + 0.1*rnd.gauss();
    //POMDPExecution(world, x, y, dual, i);
    PR2_ActionMachine(world, x, y, dual, i);
  }



#else

  ActionSystem activity;
  activity.machine->add(new CoreTasks());

  arr q,qdot;
  activity.machine->s->world.getJointState(q,qdot);
  double table_height = activity.machine->s->world.getBodyByName("table")->X.pos.z;
  cout<< "planning's table_height " <<table_height<<endl;

  uint T = 200; //time horizon

  MT::timerStart(true);

  //compute the primal and dual trajectories
  //arr heights;
  //uint numSamples = 3;
  //heights.resize(numSamples);


  double height = table_height;// + 0.1*rnd.gauss();
  //2. trajectory optimization: return primal,dual trajectories, and value functions (at each time slice)

  arr x, y, dual;
  //getTrajectory(x, y, dual, activity.machine->s->world, height, T);

  //x>>FILE("x.dat");
  //y>>FILE("y.dat");
  //dual>>FILE("z.dat");

  x<<FILE("x.dat");
  y<<FILE("y.dat");
  dual<<FILE("z.dat");

  cout<<dual<<endl;

  for(int t=0;t<x.d0;t++){
      activity.machine->s->world.setJointState(x[t]);
      activity.machine->s->world.gl().update(STRING(t), true, false, true);
  }

  cout<<"Offline Computation Time = "<< MT::realTime() <<" (s)"<<endl;


  //TESTING: Online POMDP planning
  //POMDP

  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;

  for(uint i=0;i<5;i++){
    activity.machine->s->world.setJointState(q);
    //activity.machine->s->world.gl().update(STRING(i), true, false, true);

    activity.machine->s->world.getBodyByName("table")->X.pos.z = 0.25;//table_height;// + 0.1*rnd.gauss();
    //PR2_POMDPExecution(activity, x, y, dual, i);
    PR2_ActionMachine(activity.machine->s->world, x, y, dual, i);
  }
#endif
  return 0;
}
