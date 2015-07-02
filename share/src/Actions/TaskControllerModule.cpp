#include "TaskControllerModule.h"
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>

TaskControllerModule *globalTaskControllerModule=NULL;
TaskControllerModule *taskControllerModule(){
  return globalTaskControllerModule;
}

TaskControllerModule::TaskControllerModule()
    : Module("TaskControllerModule")
    , realWorld("model.kvg")
    , feedbackController(NULL)
    , q0(realWorld.q)
    , useRos(false)
    , syncModelStateWithRos(false)
    , verbose(false) {
  modelWorld.linkToVariable(new Variable<ors::KinematicWorld>());
  modelWorld.set() = realWorld;
  feedbackController = new FeedbackMotionControl(modelWorld.set()(), true);
  globalTaskControllerModule=this;
}

TaskControllerModule::~TaskControllerModule(){
  delete feedbackController;
}

void changeColor(void*){  orsDrawColors=false; glColor(.8, 1., .8, .5); }
void changeColor2(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

#ifdef MT_ROS
void setOdom(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose){
  ors::Quaternion quat(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
  ors::Vector pos(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);

  double angle;
  ors::Vector rotvec;
  quat.getRad(angle, rotvec);
  q(qIndex+0) = pos(0);
  q(qIndex+1) = pos(1);
  q(qIndex+2) = MT::sign(rotvec(2)) * angle;
}
#endif

void TaskControllerModule::open(){
  modelWorld.get()->getJointState(q_model, qdot_model);

  feedbackController->H_rate_diag = MT::getParameter<double>("Hrate", 1.)*pr2_reasonable_W(modelWorld.set()());
  feedbackController->qitselfPD.y_ref = q0;
  feedbackController->qitselfPD.setGains(.0,10.);

  MT::open(fil,"z.TaskControllerModule");

  modelWorld.writeAccess();
  modelWorld().gl().add(changeColor);
  modelWorld().gl().add(ors::glDrawGraph, &realWorld);
  modelWorld().gl().add(changeColor2);
  modelWorld.deAccess();

  useRos = MT::getParameter<bool>("useRos",false);
  if(useRos) syncModelStateWithRos=true;
}

void TaskControllerModule::step(){
  static uint t=0;
  t++;
  if(syncModelStateWithRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
  }

  ors::Joint *trans= realWorld.getJointByName("worldTranslationRotation");

  //-- read real state
  if(useRos){
    ctrl_obs.waitForNextRevision();
    pr2_odom.waitForRevisionGreaterThan(0);
    q_real = ctrl_obs.get()->q;
    qdot_real = ctrl_obs.get()->qdot;
    if(q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N){ //we received a good reading
#ifdef MT_ROS
      setOdom(q_real, trans->qIndex, pr2_odom.get());
#endif
      realWorld.setJointState(q_real, qdot_real);
      if(syncModelStateWithRos){
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
        cout <<"** GO!" <<endl;
        syncModelStateWithRos = false;
      }
    }else{
      if(t>20){
        HALT("sync'ing real PR2 with simulated failed - using useRos=false")
      }
    }
  }
  if(syncModelStateWithRos){
    cout <<"REMOTE joint dimension=" <<q_real.N <<endl;
    cout <<"LOCAL  joint dimension=" <<realWorld.q.N <<endl;
  }

  //-- sync the model world with the AlvarMarkers
  modelWorld.writeAccess();
  AlvarMarkers alvarMarkers = ar_pose_marker.get();
  syncMarkers(modelWorld(), alvarMarkers);
//  syncMarkers(__modelWorld__, alvarMarkers); //TODO: I think this is redundant with the above (mt)
  syncMarkers(realWorld, alvarMarkers);
  modelWorld.deAccess();

  //-- display the model world (and in same gl, also the real world)
  if(!(t%5)){
    modelWorld.set()->watch(false, STRING("model world state t="<<(double)t/100.));
  }

  //-- code to output force signals
  if(true){
    ors::Shape *ftL_shape = realWorld.getShapeByName("endeffForceL");
    arr fLobs = ctrl_obs.get()->fL;
    arr uobs =  ctrl_obs.get()->u_bias;
    if(fLobs.N && uobs.N){
      arr Jft, J;
      realWorld.kinematicsPos(NoArr, J, ftL_shape->body, ftL_shape->rel.pos);
      realWorld.kinematicsPos_wrtFrame(NoArr, Jft, ftL_shape->body, ftL_shape->rel.pos, realWorld.getShapeByName("l_ft_sensor"));
      Jft = inverse_SymPosDef(Jft*~Jft)*Jft;
      J = inverse_SymPosDef(J*~J)*J;
//      MT::arrayBrackets="  ";
      fil <<t <<' ' <<zeros(3) <<' ' <<Jft*fLobs << " " <<J*uobs << endl;
//      MT::arrayBrackets="[]";
    }
  }

  //-- copy the task to the local controller
  ctrlTasks.readAccess();
  modelWorld.writeAccess();
  feedbackController->tasks = ctrlTasks();

  //-- compute the feedback controller step and iterate to compute a forward reference
  //now operational space control
  for(uint tt=0;tt<10;tt++){
    arr a = feedbackController->operationalSpaceControl();
    q_model += .001*qdot_model;
    qdot_model += .001*a;
    if(fixBase.get()) {
      qdot_model(trans->qIndex+0) = 0;
      qdot_model(trans->qIndex+1) = 0;
      qdot_model(trans->qIndex+2) = 0;
      q_model(trans->qIndex+0) = 0;
      q_model(trans->qIndex+1) = 0;
      q_model(trans->qIndex+2) = 0;
    }
    feedbackController->setState(q_model, qdot_model);
  }
  if(verbose) feedbackController->reportCurrentState();
  modelWorld.deAccess();
  ctrlTasks.deAccess();

  //-- first zero references
  CtrlMsg refs;
  refs.q =  q_model;
  refs.qdot = zeros(q_model.N);
  refs.gamma = 1.;
  refs.Kp = ARR(1.);
  refs.Kd = ARR(1.);
  refs.fL = zeros(6);
  refs.fR = zeros(6);
  refs.Ki.clear();
  refs.J_ft_inv.clear();
  refs.u_bias = zeros(q_model.N);

  //-- send base motion command
  if (!fixBase.get() && trans && trans->qDim()==3) {
    refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
    refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
    refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
  }

  //-- compute the force feedback control coefficients
  uint count=0;
  ctrlTasks.readAccess();
  feedbackController->tasks = ctrlTasks();
  for(CtrlTask *t : feedbackController->tasks) {
    if(t->active && t->f_ref.N){
      count++;
      if(count!=1) HALT("you have multiple active force control tasks - NIY");
      t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.Ki, refs.J_ft_inv, realWorld);
    }
  }
  if(count==1) refs.Kp = .5;
  ctrlTasks.deAccess();

  //-- send the computed movement to the robot
  ctrl_ref.set() = refs;
}

void TaskControllerModule::close(){
  fil.close();
}
