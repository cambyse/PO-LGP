#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/feedbackControl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <Gui/opengl.h>

VideoEncoder_libav_simple *vid;

void getTrajectory(arr& x, arr& y, arr& dual, ors::KinematicWorld& world){
  MotionProblem MP(world, false);
  x = MP.getInitialization();

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);


  Task *pos = MP.addTask("position",
                            new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target", NoVector));
  pos->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

  Task *vel = MP.addTask("position_vel", new DefaultTaskMap(posTMT, world, "endeff", NoVector));
  vel->map.order=1;
  vel->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

  Task *cons = MP.addTask("planeConstraint", new PlaneConstraint(world, "endeff", ARR(0,0,-1,.7)));
  cons->setCostSpecs(0, MP.T, {0.}, 1.);

  Task *sticky = MP.addTask("planeStickiness", new ConstraintStickiness(cons->map));
  sticky->setCostSpecs(0, MP.T, {0.}, 1.);

  //-- convert
  MotionProblemFunction MF(MP);
  Convert ConstrainedP(MF);

  //-- optimize
  mlr::timerStart();
  optConstrainedMix(x, dual, Convert(MF));
  cout <<"** optimization time = " <<mlr::timerRead() <<endl;
  MP.dualMatrix = dual;
  MP.costReport();

  if(&y){
    y.resize(x.d0, pos->map.dim_phi(world));
    for(uint t=0;t<x.d0;t++){
      world.setJointState(x[t]);
      pos->map.phi(y[t](), NoArr, world);
    }
  }
  if(&dual) dual.reshape(dual.N);
}

void testExecution(const arr& x, const arr& y, const arr& dual, ors::KinematicWorld& world, int num){
  arr q, qdot;
  world.getJointState(q, qdot);

  ofstream data(STRING("data-"<<num<<".dat"));

  ors::Shape *endeff = world.getShapeByName("endeff");
  ors::Shape *true_target = world.getShapeByName("truetarget");
  ors::Body *est_target = world.getBodyByName("target");
  ors::Body *table = world.getBodyByName("table");
  double mean_table_height = table->X.pos.z;

  double sin_jitter = mlr::getParameter<double>("sin_jitter", 0.);

  FeedbackMotionControl MC(world, false);
  MC.qitselfPD.active=true;

  //position PD task
  CtrlTask *pd_y=
      MC.addPDTask("position", .1, .8,
                   new DefaultTaskMap(posTMT, world, "endeff", NoVector, "target"));
  pd_y->prec = 10.;

  //joint space PD task
  CtrlTask *pd_x=
      MC.addPDTask("pose", 1., .8,
                    new TaskMap_qItself());
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

  //loop over time
  double tau=0.01;
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

#ifdef USE_DUAL
    //recalibrate the target based on touch
    double d=0.;
    if(pd_c->desiredApproach.y.N){ //the constraint-task tries to approach the constrained
      d = pd_c->desiredApproach.y(0); //d = true distance measured from the simulator via the constraint task map
      if(pd_c->desiredApproach.y_ref(0)==0. && d<1e-2){ //on contact
        est_target->X.pos.z = endeff->X.pos.z+0.1; //est_target position update; 0.1=known distance above table
      }
      if(est_target->X.pos.z > endeff->X.pos.z+0.1){ //also on non-contact
        est_target->X.pos.z = endeff->X.pos.z+0.1; //est_target position update
      }
    }
#endif

    //external sinus on the table height
    table->X.pos.z = mean_table_height+sin_jitter*::sin(double(t)/15);
#ifdef USE_DUAL
    plane_constraint->planeParams(3) = table->X.pos.z+0.02;
#endif

    //operational space loop
    for(uint tt=0;tt<10;tt++){
      MC.updateConstraintControllers();
      arr a = MC.operationalSpaceControl();
      q += .1*tau*qdot;
      qdot += .1*tau*a;
    }

    //display and record video
    //world.watch(false, STRING(t));
    world.gl().update(STRING(t), true, false, true);
    //flip_image(world.gl().captureImage);
    //vid->addFrame(world.gl().captureImage);

    //write data
    mlr::arrayBrackets="  ";
    data <<t <<' ' <<(t<dual.N?dual(t):0.) <<' '
        <<table->X.pos.z <<' '
       <<endeff->X.pos.z <<' '
      <<endeff->X.pos.z-table->X.pos.z <<' '
      <<est_target->X.pos.z <<' '
     <<true_target->X.pos.z <<' '
    <<endl;
  }
  data.close();

  FILE(STRING("data-"<<num<<"-err.dat")) << conv_vec2arr(true_target->X.pos)- conv_vec2arr(endeff->X.pos);
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  ors::KinematicWorld world(mlr::getParameter<mlr::String>("orsFile"));

  arr x, y, dual;
  getTrajectory(x, y, dual, world);

//  arr x2 = reverseTrajectory(x);
//  x.append(x2);
  for(uint i=0;i<1;i++) displayTrajectory(x, 1, world, "planned trajectory");
  return 0;

//  world.getBodyByName("table")->X.pos.z -= .1;
  orsDrawJoints=orsDrawProxies=orsDrawMarkers=false;
  world.setJointState(x[0]);
  world.gl().watch();

//  vid = new VideoEncoder_libav_simple("data.avi", 200./4.);
  for(uint i=0;i<10;i++){
    world.getBodyByName("table")->X.pos.z = .6 + .1*rnd.gauss();
    testExecution(x, y, dual, world, i);
  }
//  vid -> close();
//  delete vid;

  return 0;
}


