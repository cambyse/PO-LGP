#include "pd_executor_module.h"
#include <Ors/ors.h>
#include <Motion/pr2_heuristics.h>

// ############################################################################
// Executor
PDExecutor::PDExecutor()
    : world("model.kvg"), fmc(world, true), started(false), useros(false),
      limits(nullptr), collisions(nullptr),
      effPosR(nullptr), gripperR(nullptr), effOrientationR(nullptr),
      effPosL(nullptr), gripperL(nullptr), effOrientationL(nullptr)
{
  // fmc setup
  world.getJointState(q, qdot);
  fmc.H_rate_diag = pr2_reasonable_W(world);
  fmc.qitselfPD.y_ref = q;
  fmc.qitselfPD.setGains(.3, 10.);

  if(MT::getParameter<bool>("useLimits", false)) {
    limits = fmc.addPDTask("limits", 1., .8, new TaskMap_qLimits);
    limits->y_ref.setZero();
  }

  if(MT::getParameter<bool>("useCollisions", false)) {
    collisions = fmc.addPDTask("collisions", 2., .8, new ProxyTaskMap(allPTMT, {0u}, .1));
    collisions->y_ref.setZero();
    collisions->v_ref.setZero();
  }

  if(MT::getParameter<bool>("usePositionR", false)) {
    effPosR = fmc.addPDTask("MoveEffTo_endeffR", 1., .8, posTMT, "endeffR");
    effPosR->y_ref = {.4, .4, 1.2};
  }

  if(MT::getParameter<bool>("usePositionL", false)) {
    effPosL = fmc.addPDTask("MoveEffTo_endeffL", 1., .8, posTMT, "endeffL");
    effPosL->y_ref = {-.4, .4, 1.2};
  }

  if(MT::getParameter<bool>("useGripperR", false)) {
    int jointID = world.getJointByName("r_gripper_joint")->qIndex;
    gripperR = fmc.addPDTask("gripperR", .3, .8, new TaskMap_qItself(jointID, world.q.N));
    gripperR->y_ref = .08;  // open gripper 8cm
  }

  if(MT::getParameter<bool>("useGripperL", false)) {
    int jointID = world.getJointByName("l_gripper_joint")->qIndex;
    gripperL = fmc.addPDTask("gripperL", .3, .8, new TaskMap_qItself(jointID, world.q.N));
    gripperL->y_ref = .08;  // open gripper 8cm
  }

  if(MT::getParameter<bool>("useOrientationR", false)) {
    effOrientationR = fmc.addPDTask("orientationR", 1., .8, quatTMT, "endeffR", {0, 0, 0});
    effOrientationR->y_ref = {1., 0., 0., 0.};
    effOrientationR->flipTargetSignOnNegScalarProduct = true;
  }

  if(MT::getParameter<bool>("useOrientationL", false)) {
    effOrientationL = fmc.addPDTask("orientationL", 1., .8, quatTMT, "endeffL", {0, 0, 0});
    effOrientationL->y_ref = {1., 0., 0., 0.};
    effOrientationL->flipTargetSignOnNegScalarProduct = true;
  }
}

void PDExecutor::visualizeSensors() {
  arrf rh = poses_rh.get();
  if(rh.N) {
    // world.getShapeByName("sensor_rh_thumb")->rel.pos = ors::Vector(rh(0, 0), rh(0, 1), rh(0, 2));
    // world.getShapeByName("sensor_rh_index")->rel.pos = ors::Vector(rh(1, 0), rh(1, 1), rh(1, 2));
    world.getShapeByName("sensor_rh_thumb")->rel.pos = ors::Vector(rh[0]);
    world.getShapeByName("sensor_rh_index")->rel.pos = ors::Vector(rh[1]);
  }
  arrf lh = poses_lh.get();
  if(lh.N) {
    // world.getShapeByName("sensor_lh_thumb")->rel.pos = ors::Vector(lh(0, 0), lh(0, 1), lh(0, 2));
    // world.getShapeByName("sensor_lh_index")->rel.pos = ors::Vector(lh(1, 0), lh(1, 1), lh(1, 2));
    world.getShapeByName("sensor_lh_thumb")->rel.pos = ors::Vector(lh[0]);
    world.getShapeByName("sensor_lh_index")->rel.pos = ors::Vector(lh[1]);
  }
}

void PDExecutor::step() {
  if(useros && !started) {
    cout << "STARTING TO OPEN" << endl;
    initRos();
    cout << "FINISHED TO OPEN" << endl;
    started = true;
  }

  // visualize raw sensor data; not very useful anymore
  // visualizeSensors();
  world.watch(false);

  arrf cal_pose_rh = calibrated_pose_rh.get();
  arrf cal_pose_lh = calibrated_pose_lh.get();

  // only work with calibrated data
  if (cal_pose_rh.N == 0 || cal_pose_lh.N == 0)
    return;

  // cout << "============" << endl;
  // cout << "cal_pose_rh: " << cal_pose_rh << endl;
  // cout << "cal_pose_lh: " << cal_pose_lh << endl;

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
  x = cal_pose_rh(0) * 1.2;
  clip(x, 0., 1.2);
  y = cal_pose_rh(1) * 1.2;
  z = cal_pose_rh(2) * .75 - .05;
  pos = ARR(x, y, z) + ARR(0, 0, 1);
  if(effPosR) effPosR->setTarget(pos);

  // orientation
  quat = {
    (double)cal_pose_rh(3),
    (double)cal_pose_rh(4),
    (double)cal_pose_rh(5),
    (double)cal_pose_rh(6)
  };
  if(effOrientationR) effOrientationR->setTarget(quat);

  // world.getShapeByName("XXXtargetR")->rel.pos = ors::Vector(pos);
  // world.getShapeByName("XXXtargetR")->rel.rot = ors::Quaternion(quat);

  // avoid going behind your back
  x = cal_pose_lh(0) * 1.2;
  clip(x, 0., 1.2);
  y = cal_pose_lh(1) * 1.2;
  z = cal_pose_lh(2) * .75 - .05;
  pos = ARR(x, y, z) + ARR(0, 0, 1);
  if(effPosL) effPosL->setTarget(pos);

  // orientation
  quat = {
    (double)cal_pose_lh(3),
    (double)cal_pose_lh(4),
    (double)cal_pose_lh(5),
    (double)cal_pose_lh(6)
  };
  if(effOrientationL) effOrientationL->setTarget(quat);

  // world.getShapeByName("XXXtargetL")->rel.pos = ors::Vector(pos);
  // world.getShapeByName("XXXtargetL")->rel.rot = ors::Quaternion(quat);

  // set gripper
  double cal_gripper;
  cal_gripper = .09 * calibrated_gripper_rh.get();
  if(gripperR) gripperR->setTarget({cal_gripper});
  cal_gripper = .09 * calibrated_gripper_lh.get();
  if(gripperL) gripperL->setTarget({cal_gripper});

  // update fmc/ors
  double tau = 0.001;
  for (uint t = 0; t < 20; t++) {
    arr a = fmc.operationalSpaceControl();
    q += tau * qdot;
    qdot += tau * a;
    fmc.setState(q, qdot);

    if (length(qdot) < .001)
      break;
  }

  // set state
  sendRosCtrlMsg();

  // fmc.reportCurrentState();
}

void PDExecutor::sendRosCtrlMsg() {
  CtrlMsg ref;
  ref.q = q;
  arr qdotzero;
  qdotzero.resizeAs(q).setZero();
  ref.qdot = qdotzero;

  ref.fL = zeros(6);
  ref.fR = zeros(6);

  ref.Kp = {1.};
  ref.Ki.clear();
  ref.Kd = {1.};

  ref.gamma = 1.;
  ref.J_ft_inv.clear();
  ref.u_bias = zeros(q.N);

  // not actually used in controller
  // ref.velLimitRatio = .1;
  // ref.effLimitRatio = 1.;

  ctrl_ref.set() = ref;
}

void PDExecutor::initRos() {
  cout << "** Waiting for ROS message on initial configuration.." << endl;
  // get robot state from the robot
  for (;;) {
    ctrl_obs.var->waitForNextRevision();
    CtrlMsg obs = ctrl_obs.get();

    cout << "================================================\n"
         << "  observed q.N:    " << obs.q.N <<             "\n"
         << "  world q.N:       " << world.q.N  <<          "\n"
         << "  observed qdot.N: " << obs.qdot.N <<          "\n"
         << "  world qdot.N:    " << world.qdot.N <<        "\n"
         << "===============================================" << endl;

    if (obs.q.N == world.q.N && obs.qdot.N == world.qdot.N)
      break;
  }

  cout << "** Setting State of robot in simulation" << endl;
  // set robot state of the fmc
  q = ctrl_obs.get()->q;
  qdot = ctrl_obs.get()->qdot;
  fmc.setState(q, qdot);
  cout << "DONE" << endl;
}

void PDExecutor::open() {
  useros = MT::getParameter<bool>("useRos", false);
}

void PDExecutor::close() {
}
