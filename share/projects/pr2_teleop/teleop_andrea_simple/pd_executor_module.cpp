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
  fmc.qitselfPD.setGains(2., 5.);

  if(MT::getParameter<bool>("useLimits", false)) {
    limits = fmc.addPDTask("limits", .2, .8, new TaskMap_qLimits());
    // limits = fmc.addPDTask("limits", .2, .8, new TaskMap_qLimits);
    // limits->y_ref.setZero();
  }

  if(MT::getParameter<bool>("useCollisions", false)) {
    collisions = fmc.addPDTask("collisions", .5, 1.5, new ProxyTaskMap(allPTMT, {0u}, .1));
    // collisions->y_ref.setZero();
    // collisions->v_ref.setZero();
    // collisions->maxVel = -1;
    // collisions->maxAcc = -1;
  }

  if(MT::getParameter<bool>("usePositionR", false)) {
    effPosR = fmc.addPDTask("MoveEffTo_endeffR", .5, 1.5, posTMT, "endeffR");
    effPosR->y_ref = {.4, .4, 1.2};
  }

  if(MT::getParameter<bool>("usePositionL", false)) {
    effPosL = fmc.addPDTask("MoveEffTo_endeffL", .5, 1.5, posTMT, "endeffL");
    effPosL->y_ref = {-.4, .4, 1.2};
  }

  if(MT::getParameter<bool>("useGripperR", false)) {
    int jointID = world.getJointByName("r_gripper_joint")->qIndex;
    gripperR = fmc.addPDTask("gripperR", .3, .8, new TaskMap_qItself(jointID, world.q.N));
    gripperR->setTarget({.08});  // open gripper 8cm
    // gripperR->y_ref = {.08};  // open gripper 8cm
  }

  if(MT::getParameter<bool>("useGripperL", false)) {
    int jointID = world.getJointByName("l_gripper_joint")->qIndex;
    gripperL = fmc.addPDTask("gripperL", .3, .8, new TaskMap_qItself(jointID, world.q.N));
    gripperL->setTarget({.08});  // open gripper 8cm
    // gripperL->y_ref = {.08};  // open gripper 8cm
  }

  if(MT::getParameter<bool>("useOrientationR", false)) {
    effOrientationR = fmc.addPDTask("orientationR", .5, 1.5, quatTMT, "endeffR", {0, 0, 0});
    effOrientationR->y_ref = {1., 0., 0., 0.};
    effOrientationR->flipTargetSignOnNegScalarProduct = true;
  }

  if(MT::getParameter<bool>("useOrientationL", false)) {
    effOrientationL = fmc.addPDTask("orientationL", .5, 1.5, quatTMT, "endeffL", {0, 0, 0});
    effOrientationL->y_ref = {1., 0., 0., 0.};
    effOrientationL->flipTargetSignOnNegScalarProduct = true;
  }

  mid.load("g4mapping.kvg");
  transf.setZero();
  transf.addRelativeTranslation(.5+.35, -.45, .4);
  transf.addRelativeRotationDeg(-90, 0, 0, 1);
  activateTasks(false);
}

// void PDExecutor::visualizeSensors() {
//   arrf rh = poses_rh.get();
//   if(rh.N) {
//     // world.getShapeByName("sensor_rh_thumb")->rel.pos = ors::Vector(rh(0, 0), rh(0, 1), rh(0, 2));
//     // world.getShapeByName("sensor_rh_index")->rel.pos = ors::Vector(rh(1, 0), rh(1, 1), rh(1, 2));
//     world.getShapeByName("sensor_rh_thumb")->rel.pos = ors::Vector(rh[0]);
//     world.getShapeByName("sensor_rh_index")->rel.pos = ors::Vector(rh[1]);
//   }
//   arrf lh = poses_lh.get();
//   if(lh.N) {
//     // world.getShapeByName("sensor_lh_thumb")->rel.pos = ors::Vector(lh(0, 0), lh(0, 1), lh(0, 2));
//     // world.getShapeByName("sensor_lh_index")->rel.pos = ors::Vector(lh(1, 0), lh(1, 1), lh(1, 2));
//     world.getShapeByName("sensor_lh_thumb")->rel.pos = ors::Vector(lh[0]);
//     world.getShapeByName("sensor_lh_index")->rel.pos = ors::Vector(lh[1]);
//   }
// }

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

  // arrf cal_pose_rh = calibrated_pose_rh.get();
  // arrf cal_pose_lh = calibrated_pose_lh.get();

  // TODO only start executing when I press a button
 
  arr gpstate = gamepadState.get();
  CHECK(gpstate.N, "ERROR: No GamePad found");
  int button = gpstate(0);

  if(button & BTN_B)
    activateTasks(false);
  else if(button & BTN_A)
    activateTasks(true);

  teleop();
}

void PDExecutor::activateTasks(bool active) {
  cout << "Activating tasks: " << active << endl;
  if(effPosR) effPosR->active = active;
  if(gripperR) gripperR->active = active;
  if(effOrientationR) effOrientationR->active = active;

  if(effPosL) effPosL->active = active;
  if(gripperL) gripperL->active = active;
  if(effOrientationL) effOrientationL->active = active;
}

void PDExecutor::teleop() {
  poses.readAccess();
  arrf thumb_rh = mid.query(poses(), "/human/rh/thumb");
  arrf index_rh = mid.query(poses(), "/human/rh/index");
  arrf thumb_lh = mid.query(poses(), "/human/lh/thumb");
  arrf index_lh = mid.query(poses(), "/human/lh/index");
  poses.deAccess();

  // cout << "frames: " << endl;
  // cout << thumb_rh << endl;
  // cout << thumb_lh << endl;
  // cout << index_rh << endl;
  // cout << index_lh << endl;
  if (length(thumb_rh) < 1e-5 || length(index_rh) < 1e-5 ||
      length(thumb_lh) < 1e-5 || length(index_lh) < 1e-5)
    return;
  // cout << "pass" << endl;

  rigidTransf(thumb_rh);
  rigidTransf(index_rh);
  rigidTransf(thumb_lh);
  rigidTransf(index_lh);

  trackHand(thumb_rh, index_rh, effPosR, gripperR, effOrientationR);
  trackHand(thumb_lh, index_lh, effPosL, gripperL, effOrientationL);

  // op space control loop
  runOperationalSpaceControl();

  // set state
  sendRosCtrlMsg();
}

void PDExecutor::rigidTransf(arrf &pose) {
  ors::Transformation T;

  ors::Vector v(pose(0), pose(1), pose(2));
  ors::Quaternion q(pose(3), pose(4), pose(5), pose(6));
  // T.appendInvTransformation(transf_mocap_robot);

  // T.addRelativeTranslation(.55, .5, 1.);
  // T.appendTransformation(transf);

  // T = T * transf;

  // cout << "before: " << v << " " << q << endl;

  v = transf * v;
  q = transf.rot * q;

  // cout << "after: " << v << " " << q << endl;

  pose(0) = v.x;
  pose(1) = v.y;
  pose(2) = v.z;
  pose(3) = q.w;
  pose(4) = q.x;
  pose(5) = q.y;
  pose(6) = q.z;
}

void PDExecutor::trackHand(const arrf &thumb, const arrf &index, CtrlTask *effPos, CtrlTask *gripper, CtrlTask *effOrientation) {
  // Setting hand position
  // cout << "thumb: " << thumb << endl;
  // cout << "thumb.dim(): " << thumb.dim() << endl;

  arrf pos = .5f * (thumb.subRange(0, 2) + index.subRange(0, 2));
  if(effPos) effPos->setTarget({(double)pos(0), (double)pos(1), (double)pos(2)});

  // Setting hand orientation
  arr quat = makeHandOrientation(thumb, index);
  if(effOrientation) effOrientation->setTarget(quat);

  // Setting gripper
  double cal_gripper = length(thumb.subRange(0, 2) - index.subRange(0, 2)) - .04;
  clip(cal_gripper, 0., .08);
  if(gripper) gripper->setTarget({cal_gripper});
}

arr PDExecutor::makeHandOrientation(const arrf &thumb, const arrf &index) {
  ors::Quaternion quat;
  ors::Vector x_thumb, x_index;
  ors::Vector pos_thumb, pos_index;
  ors::Vector x_pr2, y_pr2, z_pr2;

  pos_thumb.set(thumb(0), thumb(1), thumb(2));
  quat.set(thumb(3), thumb(4), thumb(5), thumb(6));
  x_thumb = quat * Vector_x;

  pos_index.set(index(0), index(1), index(2));
  quat.set(index(3), index(4), index(5), index(6));
  x_index = quat * Vector_x;

  y_pr2 = pos_index - pos_thumb;
  y_pr2.normalize();

  x_pr2 = .5 * (x_index + x_thumb);
  x_pr2.makeNormal(y_pr2);
  x_pr2.normalize();

  z_pr2 = x_pr2 ^ y_pr2;
  z_pr2.normalize();
  
  double matrix[9];
  matrix[0] = x_pr2.x;
  matrix[1] = y_pr2.x;
  matrix[2] = z_pr2.x;
  matrix[3] = x_pr2.y;
  matrix[4] = y_pr2.y;
  matrix[5] = z_pr2.y;
  matrix[6] = x_pr2.z;
  matrix[7] = y_pr2.z;
  matrix[8] = z_pr2.z;
  quat.setMatrix(matrix);

  return {quat.w, quat.x, quat.y, quat.z};
}

void PDExecutor::runOperationalSpaceControl() {
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

