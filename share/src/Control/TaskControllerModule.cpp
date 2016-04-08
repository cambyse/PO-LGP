#include "TaskControllerModule.h"
//#include <RosCom/rosalvar.h> //todo: don't deal with the markers here! ObjectFilter..
#include <Gui/opengl.h>
#include <RosCom/baxter.h>

void lowPassUpdate(arr& lowPass, const arr& signal, double rate=.1){
  if(lowPass.N!=signal.N){ lowPass=zeros(signal.N); return; }
  lowPass = (1.-rate)*lowPass + rate*signal;
}

TaskControllerModule::TaskControllerModule(const char* _robot)
  : Module("TaskControllerModule", .01)
  , taskController(NULL)
  , oldfashioned(true)
  , useRos(false)
  , requiresInitialSync(true)
  , syncModelStateWithReal(false)
  , verbose(false)
  , useDynSim(true){

  useRos = mlr::getParameter<bool>("useRos",false);
  oldfashioned = mlr::getParameter<bool>("oldfashinedTaskControl", true);
  useDynSim = !oldfashioned && !useRos; //mlr::getParameter<bool>("useDynSim", true);

  robot = mlr::getParameter<mlr::String>("robot", _robot);
  if(robot=="pr2") realWorld.init(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  else if(robot=="baxter") realWorld.init(mlr::mlrPath("data/baxter_model/baxter.ors").p);
  else HALT("undefined robot '" <<robot <<"'");
  q0 = realWorld.q;

}

TaskControllerModule::~TaskControllerModule(){
}

void changeColor(void*){  orsDrawColors=false; glColor(.5, 1., .5, .7); }
void changeColor2(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

void TaskControllerModule::open(){
  modelWorld.set() = realWorld;
  taskController = new TaskController(modelWorld.set()(), true);

  modelWorld.get()->getJointState(q_model, qdot_model);

  taskController->qNullCostRef.y_ref = q0;
  taskController->qNullCostRef.setGains(0., 1.);
  taskController->qNullCostRef.prec = mlr::getParameter<double>("Hrate", 1.)*modelWorld.get()->getHmetric();

#if 1
  modelWorld.writeAccess();
  modelWorld().gl().add(changeColor);
  modelWorld().gl().add(ors::glDrawGraph, &realWorld);
  modelWorld().gl().add(changeColor2);
  modelWorld.deAccess();
#endif

  if(useRos || !oldfashioned) syncModelStateWithReal=true;

  if(!oldfashioned && !useRos) {
    dynSim = new RTControllerSimulation(0.01, false, 0.);
    dynSim->threadLoop();
  }

  dataFiles.open({"q", "qdot", "qddot", "q_low", "qdot_low", "qddot_low", "qddot_des"});
}


void TaskControllerModule::step(){
  static uint t=0;
  t++;

  ors::Joint *trans= realWorld.getJointByName("worldTranslationRotation", false);

  //-- read real state
  if(useRos || !oldfashioned){
    bool succ=true;
    qdot_last = qdot_real;
    if(robot=="pr2"){
      ctrl_obs.waitForRevisionGreaterThan(0);
      if(useRos)  pr2_odom.waitForRevisionGreaterThan(0);
      q_real = ctrl_obs.get()->q;
      qdot_real = ctrl_obs.get()->qdot;
      q_real.subRef(trans->qIndex, trans->qIndex+2) = pr2_odom.get();
    }
    if(robot=="baxter"){
      jointState.waitForRevisionGreaterThan(20);
      q_real = realWorld.q;
      succ = baxter_update_qReal(q_real, jointState.get(), realWorld);
      qdot_real = zeros(q_real.N);
    }
    ctrl_q_real.set() = q_real;
    if(succ && q_real.N==realWorld.q.N && qdot_real.N==realWorld.q.N){ //we received a good reading
      realWorld.setJointState(q_real, qdot_real);
      if(syncModelStateWithReal){
        q_model = q_real;
        qdot_model = qdot_real;
        modelWorld.set()->setJointState(q_model, qdot_model);
        q_history.prepend(q_real); q_history.reshape(q_history.N/q_real.N, q_real.N);
        if(q_history.d0>5) q_history.resizeCopy(5, q_real.N);

        if(q_history.d0>0) lowPassUpdate(q_lowPass, q_history[0]);
        if(q_history.d0>1) lowPassUpdate(qdot_lowPass, (q_history[0]-q_history[1])/.01);
        if(q_history.d0>2) lowPassUpdate(qddot_lowPass, (q_history[0]-2.*q_history[1]+q_history[2])/(.01*.01));
        if(oldfashioned) syncModelStateWithReal = false;
      }
      requiresInitialSync = false;
    }else{
      cout <<"** Waiting for ROS message on initial configuration.." <<endl;
      if(t>20){
        HALT("sync'ing real robot with simulated failed")
      }
    }
  }

  //-- sync the model world with the AlvarMarkers
//  modelWorld.writeAccess();
//  AlvarMarkers alvarMarkers = ar_pose_marker.get();
//  syncMarkers(modelWorld(), alvarMarkers);
//  syncMarkers(realWorld, alvarMarkers);
//  modelWorld.deAccess();

  //-- display the model world (and in same gl, also the real world)
  if(!(t%5)){
#if 1
    //modelWorld.set()->watch(false, STRING("model world state t="<<(double)t/100.));
#endif
  }

  //-- compute the feedback controller step and iterate to compute a forward reference
  CtrlMsg refs;
  if(oldfashioned){

    //now operational space control
    ctrlTasks.readAccess();
    modelWorld.writeAccess();
    taskController->tasks = ctrlTasks();
    for(uint tt=0;tt<10;tt++){
      arr a = taskController->operationalSpaceControl();
      q_model += .001*qdot_model;
      qdot_model += .001*a;
      if(trans && fixBase.get()) {
        qdot_model(trans->qIndex+0) = 0;
        qdot_model(trans->qIndex+1) = 0;
        qdot_model(trans->qIndex+2) = 0;
        //      q_model(trans->qIndex+0) = 0;
        //      q_model(trans->qIndex+1) = 0;
        //      q_model(trans->qIndex+2) = 0;
      }
      taskController->setState(q_model, qdot_model);
    }
    if(verbose) taskController->reportCurrentState();
    modelWorld.deAccess();
    ctrlTasks.deAccess();

    ctrl_q_ref.set() = q_model;

    //-- first zero references
    refs.q =  q_model;
    refs.qdot = zeros(q_model.N);
    refs.fL_gamma = 1.;
    refs.Kp = ARR(1.);
    refs.Kd = ARR(1.);
    refs.Ki = ARR(0.2);
    refs.fL = zeros(6);
    refs.fR = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q_model.N);
    refs.intLimitRatio = 0.7;

    //-- compute the force feedback control coefficients
    uint count=0;
    ctrlTasks.readAccess();
    taskController->tasks = ctrlTasks();
    for(CtrlTask *t : taskController->tasks) {
      if(t->active && t->f_ref.N){
        count++;
        if(count!=1) HALT("you have multiple active force control tasks - NIY");
        t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.KiFTL, refs.J_ft_invL, realWorld);
      }
    }
    if(count==1) refs.Kp = .5;
    ctrlTasks.deAccess();

  }else{

    ctrlTasks.readAccess();
    modelWorld.writeAccess();
    taskController->tasks = ctrlTasks();

#if 0
    arr u_bias, Kp, Kd;
    arr M, F;
    feedbackController->world.equationOfMotion(M, F, false);
    arr u_mean = feedbackController->calcOptimalControlProjected(Kp, Kd, u_bias, M, F); // TODO: what happens when changing the LAWs?
#else

    //-- compute desired acceleration law in q-space
    arr a, Kp, Kd, k, JCJ;
    a = taskController->getDesiredLinAccLaw(Kp, Kd, k, JCJ);
    checkNan(k);

    //-- limit the step
    arr q_step = pseudoInverse(Kp)*(k-Kp*q_real);
    clip(q_step, -.1, .1);
    arr q_ref = q_real + q_step;

    //-- translate to motor torques
#if 0
    arr M, F;
    taskController->world.equationOfMotion(M, F, false);
//    arr u_bias = M*k + F;
    Kp = M*Kp;
    Kd = M*Kd;
    checkNan(Kp);
#endif

    //-- compute the error between expected change in velocity and true one
#if 0
    if(!a_last.N) a_last = a;
    if(!qdot_last.N) qdot_last = qdot_real;
    arr a_err = (qdot_real - qdot_last)/.01 - a_last;
    a_last = a;
    // integrate this error
    if(!aErrorIntegral.N) aErrorIntegral = JCJ * a_err;
    else aErrorIntegral += a_err;
    // add integral error to control bias
    u_bias -= .01 * M * aErrorIntegral;
#endif
#endif

    // F/T limit control
    arr K_ft, J_ft_inv, fRef;
    double gamma;
    taskController->calcForceControl(K_ft, J_ft_inv, fRef, gamma);


    if(verbose){
      LOG(0) <<"************** Tasks Report **********";
      taskController->reportCurrentState();
    }

//    dataFiles.write({&modelWorld().q, &modelWorld().qdot, &qddot, &q_lowPass, &qdot_lowPass, &qddot_lowPass, &aErrorIntegral});

    modelWorld.deAccess();
    ctrlTasks.deAccess();

    refs.q = q_ref;
    refs.qdot = zeros(q_model.N);
    refs.fL_gamma = gamma;
    refs.Kp = Kp;
    refs.Kd = Kd;
    refs.Ki.clear();
    refs.fL = fRef;
    refs.fR = zeros(6);
    refs.KiFTL = K_ft;
    refs.J_ft_invL = J_ft_inv;
    refs.u_bias = zeros(q_ref.N); //u_bias;
    refs.intLimitRatio = 0.7;
    refs.qd_filt = .99;
  }

  ctrl_q_ref.set() = refs.q;

  //-- send base motion command
  if(useRos) {
    if (!fixBase.get() && trans && trans->qDim()==3) {
      refs.qdot(trans->qIndex+0) = qdot_model(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot_model(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot_model(trans->qIndex+2);
    }
  }

  //-- send the computed movement to the robot
  if(!requiresInitialSync && (useRos || useDynSim)){
    ctrl_ref.set() = refs;
  }
}

void TaskControllerModule::close(){
  delete taskController;
}