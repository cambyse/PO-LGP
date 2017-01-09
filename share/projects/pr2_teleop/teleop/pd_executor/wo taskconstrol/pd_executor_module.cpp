#include "pd_executor_module.h"
#include <Ors/ors.h>



//#ifdef WITH_ROS
  //  #include <RosCom/roscom.h>
//#endif

// ############################################################################
// Executor
PDExecutor::PDExecutor()
    : world("model.kvg"), fmc(world, true), useros(false),
      limits(nullptr), collision(nullptr),
      effPosR(nullptr), gripperR(nullptr), effOrientationR(nullptr),
      effPosL(nullptr), gripperL(nullptr), effOrientationL(nullptr)
{
  // fmc setup
  world.getJointState(q, qdot);
  fmc.H_rate_diag = world.getHmetric();
  fmc.qitselfPD.y_ref = q;
  fmc.qitselfPD.setGains(2.3,10);
 // fmc.qitselfPD.active = false;
  fmc.qitselfPD.prec = 0.100;
  //fmc.qitselfPD.maxVel = 0.008;
  //fmc.qitselfPD.maxAcc = 0.09;
  //fmc.qitseldPD.f_Igain = 1.;

  if(mlr::getParameter<bool>("useLimits", false)) {
    limits = fmc.addPDTask("limits", 0.2, .8, new TaskMap_qLimits);
   // limits->y_ref.setZero();

  limits->prec = 0.1;
  }

  if(mlr::getParameter<bool>("useCollisions", false)) {
    collision = fmc.addPDTask("collision", 0.1, 5.8, new TaskMap_Proxy(allPTMT, {0u}, .1));
  }

  if(mlr::getParameter<bool>("usePositionR", false)) {
    effPosR = fmc.addPDTask("MoveEffTo_endeffR", .2, 1.8, posTMT, "endeffR");
    effPosR->y_ref = {0.8, -.5, 1.};
    //effPosR->maxVel = 0.004;
  }

  if(mlr::getParameter<bool>("usePositionL", false)) {
    effPosL = fmc.addPDTask("MoveEffTo_endeffL", .2, 1.8, posTMT, "endeffL");
    effPosL->y_ref = {0.8, .5, 1.};
    //effPosL->maxVel = 0.004;
  }

  if(mlr::getParameter<bool>("useGripperR", false)) {
    int jointID = world.getJointByName("r_gripper_joint")->qIndex;
    gripperR = fmc.addPDTask("gripperR", .3, 1.8, new TaskMap_qItself(jointID, world.q.N));
    gripperR->setTarget({0.01});
    //gripperR->y_ref = {.08};  // open gripper 8cm
  }

  if(mlr::getParameter<bool>("useGripperL", false)) {
    int jointID = world.getJointByName("l_gripper_joint")->qIndex;
    gripperL = fmc.addPDTask("gripperL", .3, 1.8, new TaskMap_qItself(jointID, world.q.N));
    gripperL->setTarget({0.01});
    //gripperL->y_ref = {.08};  // open gripper 8cm
  }

  if(mlr::getParameter<bool>("useOrientationR", false)) {
    effOrientationR = fmc.addPDTask("orientationR", .2, 1.8, quatTMT, "endeffR", {0, 0, 0});
    effOrientationR->y_ref = {1., 0., 0., 0.};
    effOrientationR->flipTargetSignOnNegScalarProduct = true;

  }

  if(mlr::getParameter<bool>("useOrientationL", false)) {
    effOrientationL = fmc.addPDTask("orientationL", .2,1.8, quatTMT, "endeffL", {0, 0, 0});
    effOrientationL->y_ref = {1., 0., 0., 0.};
    effOrientationL->flipTargetSignOnNegScalarProduct = true;

  }

  if(mlr::getParameter<bool>("fc", false)) {
    fc = fmc.addConstraintForceTask("test", new PairCollisionConstraint(world,"endeffForceR","r_ft_sensor"));
  
    //fc->desiredApproach.f_ref = {1.,0.,0.,0.,0.,0.};
    //fc->desiredApproach.v_ref.clear();// = {0.,0.,0.};
    //fc->desiredApproach.y_ref.clear();// = {0.,0.,0.};

    fc->desiredApproach.active= true;
    fc->desiredApproach.Dgain = 1.;
    fc->desiredApproach.Pgain = 1.;
    fc->desiredApproach.f_Igain = 1.;
    fc->desiredForce = 1000.;
      fc->active = true;
  }

}

void PDExecutor::visualizeSensors()
{
  floatA rh = poses_rh.get();
  if(rh.N) {
    world.getShapeByName("sensor_rh_thumb")->rel.pos = mlr::Vector(rh(0, 0), rh(0, 1), rh(0, 2));
    world.getShapeByName("sensor_rh_index")->rel.pos = mlr::Vector(rh(1, 0), rh(1, 1), rh(1, 2));
  }
  floatA lh = poses_lh.get();
  if(lh.N) {
    world.getShapeByName("sensor_lh_thumb")->rel.pos = mlr::Vector(lh(0, 0), lh(0, 1), lh(0, 2));
    world.getShapeByName("sensor_lh_index")->rel.pos = mlr::Vector(lh(1, 0), lh(1, 1), lh(1, 2));
  }
}

void PDExecutor::step()
{
    cout<<"\x1B[2J\x1B[H";
  if (useros && !inited) {
    cout << "STARTING TO OPEN" << endl;
    initRos();
    cout << "FINISHED TO OPEN" << endl;
    inited = true;
  }

  // visualize raw sensor data; not very useful anymore
  // visualizeSensors();
   world.watch(false);



  floatA cal_pose_rh = calibrated_pose_rh.get();
  floatA cal_pose_lh = calibrated_pose_lh.get();

  bool init;
  init = initmapper.get();
  if(init)
  {
    effPosR->active = false;
    effPosL->active = false;
    effOrientationR->active = false;
    effOrientationL->active = false;
    gripperL->active = false;
    gripperR->active = false;
  //  fc->active = false;
  }
  else
  {

    effPosR->active = true;
    effPosL->active = true;
    effOrientationR->active = true;
    effOrientationL->active = true;
    gripperL->active = true;
    gripperR->active = true;
    fc->active = true;
  }

   CtrlMsg obs = ctrl_obs.get();
    cout<<obs.fL<<endl<<obs.fR<<endl;
 world.getShapeByName("endeffForceR")->rel.pos = mlr::Vector(.01/*-obs.fR(0)*/,.01 /*-obs.fR(1)*/,.01 /*-obs.fR(2)*/);

// world.getShapeByName("endeffForceR")->size[2]= mlr::Vector(obs.fR(0), obs.fR(1), obs.fR(2)).length();


/*  // only work with calibrated data
  if (cal_pose_rh.N == 0 || cal_pose_lh.N == 0)
    return;
*/
  // cout << "============" << endl;
  // cout << "cal_pose_rh: " << cal_pose_rh << endl;

  // cout << "cal_pose_lh: " << cal_pose_lh << endl;
if(!init)
{
  // set arm poses
  double x, y, z;
  arr pos, quat;

  // avoid going behind your back
  // x = clip(cal_pose_rh(0) * 1.1, 0., 1.1);
  // y = cal_pose_rh(1) * 1.1;
  // z = cal_pose_rh(2) * .75 - .05;
  // pos = { x, y, z };
  // pos_shoulder_frame = pos + ARR(.4, -.1, 1.0);
  // effPosR->setTarget(pos_shoulder_frame);
  // x = clip(cal_pose_rh(0) * 1.2, 0., 1.2);
  //mlr::Vector baseV = world.getShapeByName("base_footprint")->rel.pos;
  //mlr::Quaternion baseQ = world.getShapeByName("base_footprint")->rel.rot;
  x = cal_pose_rh(0) * 1;
 // clip(x, 0., 1.2);
  y = cal_pose_rh(1) * 1;
  z = cal_pose_rh(2) * 1;
  pos = ARR(x, y, z) + ARR(0.6, 0., 1.);
  if(effPosR) effPosR->setTarget(pos);

  // orientation
  quat = {
    (double)cal_pose_rh(3),
    (double)cal_pose_rh(4),
    (double)cal_pose_rh(5),
    (double)cal_pose_rh(6)
  };
  if(effOrientationR) effOrientationR->setTarget(quat);

//  world.getShapeByName("XXXtargetR")->rel.pos = mlr::Vector(pos);
//  world.getShapeByName("XXXtargetR")->rel.rot = mlr::Quaternion(quat);

  // avoid going behind your back
  // x = clip(cal_pose_lh(0) * 1.2, 0., 1.2);
  x = cal_pose_lh(0) * 1;
  // clip(x, 0., 1.2);
  y = cal_pose_lh(1) * 1;
  z = cal_pose_lh(2) * 1;
  pos = ARR(x, y, z) + ARR(0.6, 0., 1.);
  if(effPosL) effPosL->setTarget(pos);

  // orientation
  quat = {
    (double)cal_pose_lh(3),
    (double)cal_pose_lh(4),
    (double)cal_pose_lh(5),
    (double)cal_pose_lh(6)
  };
  if(effOrientationL) effOrientationL->setTarget(quat);

 // world.getShapeByName("XXXtargetL")->rel.pos = mlr::Vector(pos);
 // world.getShapeByName("XXXtargetL")->rel.rot = mlr::Quaternion(quat);

  // set gripper
  double cal_gripper;
  cal_gripper =  calibrated_gripper_rh.get();
  if(gripperR) gripperR->setTarget({cal_gripper});
  cal_gripper =  calibrated_gripper_lh.get();
  if(gripperL) gripperL->setTarget({cal_gripper});
}
  // update fmc/ors

 double tau = 0.001;
 // arr a = fmc.operationalSpaceControl();

  for (uint t = 0; t < 20 ; t++) {
    fmc.updateConstraintControllers();
    arr a = fmc.operationalSpaceControl();
    q += tau * qdot;
    qdot += tau * a;
    fmc.setState(q, qdot);

  
   // if (length(qdot) < .001)
   //   break;
  }
 // cout<<q<<endl<<qdot<<endl; 

  // set state
  sendRosCtrlMsg();//testing
     

   fmc.reportCurrentState();
}

void PDExecutor::sendRosCtrlMsg()
{
  CtrlMsg ref;
  ref.q = q;
  arr qdotzero;
  qdotzero.resizeAs(q).setZero();
  ref.qdot = qdotzero;

  ref.u_bias = zeros(q.N);

  ref.fL =zeros(6);
  ref.fR =zeros(6);
  ref.Kp = {1.};
  ref.Ki.clear();
  ref.Kd = {1.};
  ref.gamma = 1.;
  ref.J_ft_inv.clear();
  ctrl_ref.set() = ref;
 //  roscom->publishJointReference();
//#endif
}

void PDExecutor::initRos()
{
//#ifdef WITH_ROS
  // if (roscom == nullptr)
  //   return;

  cout << "** Waiting for ROS message on initial configuration.." << endl;
  // get robot state from the robot
  for (;;) {
    ctrl_obs.data->waitForNextRevision();
    CtrlMsg obs = ctrl_obs.get();

    cout << "================================================\n"
         << "  observed q.N:    " << obs.q.N << "\n"
         << "  world q.N:       " << world.q.N  << "\n"
         << "  observed qdot.N: " << obs.qdot.N << "\n"
         << "  world qdot.N:    " << world.qdot.N << "\n"
         << "================================================" << endl;

    if (obs.q.N == world.q.N && obs.qdot.N == world.qdot.N)
      break;

  }

  cout << "** Setting State of robot in simulation" << endl;
  // set robot state of the fmc
  q = ctrl_obs.get()->q;
  qdot = ctrl_obs.get()->qdot;
  fmc.setState(q, qdot);
  cout << "DONE" << endl;
//#endif
}

void PDExecutor::open()
{
  useros = mlr::getParameter<bool>("useRos", false);
}

void PDExecutor::close()
{
}
