//#include <Motion/gamepad2tasks.h>
//#include <Motion/feedbackControl.h>
//#include <Hardware/gamepad/gamepad.h>
//#include <Motion/pr2_heuristics.h>
//#include <pr2/roscom.h>
//#include <pr2/rosmacro.h>
//#include <pr2/rosalvar.h>
//#include <pr2/trajectoryInterface.h>
//#include <Core/util.h>

#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <pr2/trajectoryInterface.h>

void changeAlpha(void*){  orsDrawAlpha = 1.; }

arr makeJacobian(arr &JPos, arr &JAngle) {
  arr J = zeros(JPos.d0+JAngle.d0,JPos.d1);
  for(uint i = 0; i < JPos.d1; i++) {
    for(uint j = 0; j < JPos.d0; j++) {
      J(j,i) = JPos(j,i);
    }
    for(uint j = 0; j < JAngle.d0; j++) {
      J(j+JPos.d0,i) = JAngle(j,i);
    }
  }
  return J;
}

arr makeVec(arr &posVec, arr &angleVec) {
  arr vec = zeros(posVec.d0+angleVec.d0);
  for(uint i = 0; i < posVec.d0; i++) {
    vec(i) = posVec(i);
  }
  for(uint i = 0; i < angleVec.d0; i++) {
    vec(i+posVec.d0) = angleVec(i);
  }
  return vec;
}


arr generateTrajectoryForTable() {
  ors::KinematicWorld world("model.kvg");

  MotionProblem MP(world);

  arr q;
  arr qDot;
  MP.world.getJointState(q, qDot);


  uint contactT = MP.T/2;
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(MP.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("dircetion", new DefaultTaskMap(vecDiffTMT, MP.world, "endeffL", ors::Vector(1.0,0.0,0.0), "startShape", ors::Vector(0.0,0.0,-1.0)));
  t->setCostSpecs(contactT-2, MP.T, {0.0}, 10.0);

  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"startShape", NoVector));
  t->setCostSpecs(contactT-2, contactT, {0.0}, 1000.0);
  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"endShape", NoVector));
  t->setCostSpecs(MP.T-2, MP.T, {0.0}, 1000.0);

  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MF(MP);

  arr traj = MP.getInitialization();

  optConstrainedMix(traj, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  MP.costReport();

  return traj;
}

void displayTrajectory() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);
  arr traj = generateTrajectoryForTable();

  world.watch(true);
  for(uint i = 1; i < traj.d0-1; i++) {
    mlr::wait(0.05);
    world.setJointState(traj[i]);
    world.watch(false);
  }
}

void exTrajectoryOnPr2() {
  ors::KinematicWorld realWorld("model_pr2.kvg");
  makeConvexHulls(realWorld.shapes);

  TrajectoryInterface *ti = new TrajectoryInterface(realWorld);

  ti->world->gl().add(changeAlpha);
  arr trajSimulation = generateTrajectoryForTable();
  arr trajWorld;

  transferQbetweenTwoWorlds(trajWorld,trajSimulation,realWorld, ors::KinematicWorld("model.kvg"));

  ti->executeTrajectory(trajWorld,10.0);
}



void controlJointSpace() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);
  arr traj = generateTrajectoryForTable();

  world.watch(true);

  arr qRef,qDotRef;
  arr q,qDot;
  arr M,F;
  arr x,xAngle,J,JAngle,xRef,xAngleRef;

  world.getJointState(q, qDot);

  arr Kp = eye(q.d0)*50.0;
  arr Kd = eye(q.d0)*20.0;

  arr u;

  double tau = 0.01;

  for(uint i = 1; i < traj.d0-1; i++) {
    mlr::wait(0.01);

    qRef = traj[i];
    qDotRef = zeros(q.d0);


    for(uint t = 0; t < 50; t++) {
      world.equationOfMotion(M,F);
      world.kinematicsPos(x, J, world.getBodyByName("l_gripper_l_finger_link"));
      world.kinematicsQuat(xAngle, JAngle, world.getBodyByName("l_gripper_l_finger_link"));
      world.getJointState(q, qDot);

      world.setJointState(qRef,qDotRef);
      world.kinematicsPos(xRef, NoArr, world.getBodyByName("l_gripper_l_finger_link"));
      world.kinematicsQuat(xAngleRef, NoArr, world.getBodyByName("l_gripper_l_finger_link"));
      world.setJointState(q,qDot);

      u = M*(Kp*(qRef-q)+Kd*(qDotRef-qDot)) + F;

      if(i > 30 && i < 40) {
        arr force = ARR(-10.0,0.0,0.0);
        u += ~J*force;
        cout << "force" << endl;
      }

      world.stepDynamics(u, tau, 0.0);
      world.watch(false);
    }
  }
  world.watch(true);
}




void controlStiffnessSpace() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);
  arr traj = generateTrajectoryForTable();

  world.watch(true);

  arr qRef,qDotRef;
  arr q,qDot;
  arr M,F;
  arr x,xAngle,J,JAngle,xRef,xAngleRef;

  world.getJointState(q, qDot);

  arr Kp = eye(7)*30.0;
  Kp(2,2) = 30.0;
  Kp(3,3) = 0.0;
  Kp(4,4) = 00.0;
  Kp(5,5) = 00.0;
  Kp(6,6) = 0.0;
  arr Kd = eye(q.d0)*30.0;

  arr u;

  double tau = 0.01;

  for(uint i = 1; i < traj.d0-1; i++) {
    mlr::wait(0.01);

    qRef = traj[i];
    qDotRef = zeros(q.d0);


    for(uint t = 0; t < 50; t++) {
      world.equationOfMotion(M,F);
      world.kinematicsPos(x, J, world.getBodyByName("l_gripper_l_finger_link"));
      world.kinematicsQuat(xAngle, JAngle, world.getBodyByName("l_gripper_l_finger_link"));
      world.getJointState(q, qDot);

      world.setJointState(qRef,qDotRef);
      world.kinematicsPos(xRef, NoArr, world.getBodyByName("l_gripper_l_finger_link"));
      world.kinematicsQuat(xAngleRef, NoArr, world.getBodyByName("l_gripper_l_finger_link"));
      world.setJointState(q,qDot);

      arr JTilde = makeJacobian(J,JAngle);
      arr xTilde = makeVec(x,xAngle);
      arr xTildeRef = makeVec(xRef,xAngleRef);

      //u = M*(~JTilde*Kp*(xTildeRef-xTilde)+Kd*(qDotRef-qDot)) + F;
      double k = 10.0-5.0/25.0*i;
      if(k < 3.0) {
        k = 100.0;
      }
      //k = 0.0;
      JTilde = eye(q.d0)*k + ~JTilde*Kp*JTilde;
      u = M*(JTilde*(qRef-q) + Kd*(qDotRef-qDot)) + F;

      if(i > 30 && i < 40) {
        arr force = ARR(-0.0,0.0,0.0);
        u += ~J*force;
        cout << "force" << endl;
      }

      world.stepDynamics(u, tau, 0.0);
      world.watch(false);
    }
  }
  world.watch(true);
}





void controlOperationalSpace() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);



  MotionProblem MP(world);

  uint contactT = MP.T/2;
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(MP.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("dircetion", new DefaultTaskMap(vecDiffTMT, MP.world, "endeffL", ors::Vector(1.0,0.0,0.0), "startShape", ors::Vector(0.0,0.0,-1.0)));
  t->setCostSpecs(contactT-2, MP.T, {0.0}, 10.0);

  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"startShape", NoVector));
  t->setCostSpecs(contactT-2, contactT, {0.0}, 1000.0);
  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"endShape", NoVector));
  t->setCostSpecs(MP.T-2, MP.T, {0.0}, 1000.0);

  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MF(MP);

  arr traj = MP.getInitialization();

  optConstrainedMix(traj, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  //MP.costReport();
  arr trajDot,trajDDot;

  double tau = 0.01;

  getVel(trajDot,traj,tau);
  getAcc(trajDDot,traj,tau);

  world.watch(true);

  arr qRef,qDotRef;
  arr q,qDot;
  arr M,F;
  arr x,xAngle,J,JAngle,xRef,xAngleRef;

  world.getJointState(q, qDot);

  arr u;


  arr T,TSharp;
  arr H = eye(q.d0)*1.0;
  arr C = eye(6)*10.0;
  arr yDDotDes;

  arr Kp = eye(6)*10.0;
  Kp(2,2) = 10.0;
  Kp(3,3) = 2.0;
  Kp(4,4) = 2.0;
  Kp(5,5) = 2.0;
  arr Kd = eye(q.d0)*10.0;

  arr trajOld;

  for(uint i = 1; i < 10000; i++) {
    mlr::wait(0.01);
    if(i < traj.d0) {
      qRef = traj[i];
      trajOld = traj[i-1];
    } else {
      qRef = traj[traj.d0-1];
      trajOld = qRef;
    }

    qDotRef = zeros(q.d0);//trajDot[i];


    for(uint t = 0; t < 50; t++) {
      world.equationOfMotion(M,F);
      world.kinematicsPos(x, J, world.getBodyByName("l_gripper_l_finger_link"));
      world.kinematicsVec(xAngle, JAngle, world.getBodyByName("l_gripper_l_finger_link"),ors::Vector(-1.0,.0,.0));
      world.jacobianR(JAngle,world.getBodyByName("l_gripper_l_finger_link"));
      world.getJointState(q, qDot);

      world.setJointState(qRef,qDotRef);
      world.kinematicsPos(xRef, NoArr, world.getBodyByName("l_gripper_l_finger_link"));
      world.kinematicsVec(xAngleRef, NoArr, world.getBodyByName("l_gripper_l_finger_link"),ors::Vector(-1.0,.0,.0));
      world.setJointState(q,qDot);

      xRef = makeVec(xRef,xAngleRef);
      x = makeVec(x,xAngle);
      arr JTilde = makeJacobian(J,JAngle);
      //arr xTilde = makeVec(x,xAngle);
      //arr xTildeRef = makeVec(xRef,xAngleRef);

      //T = J*inverse(M);
      //TSharp = inverse(~T*C*T+H)*~T*C;
      //yDDotDes = Kp*(xRef-x) + Kd*J*(qDotRef-qDot);
      J = JTilde;
      arr x0Pos,x0Angle;
      world.setJointState(qRef,zeros(q.d0));
      world.kinematicsPos(x0Pos, NoArr, world.getBodyByName("l_gripper_l_finger_link"));
      world.kinematicsVec(x0Angle, NoArr, world.getBodyByName("l_gripper_l_finger_link"),ors::Vector(-1.0,.0,.0));
      world.setJointState(q,qDot);
      arr x0 = makeVec(x0Pos,x0Angle);

      //arr k = Kp*(x0);
      arr k = zeros(6);
      //cout << k << endl;
      arr A = ~J*C*J + ~M*H*M;
      //u = M*(inverse(A)*(~M*H*(M*(eye(q.d0)*3.0*(qRef-q)+eye(q.d0)*1.0*(qDotRef-qDot))+F-F)+~J*C*(Kp*J*(qRef-q)+k)) + Kd*(qDotRef-qDot))+F;// + M*(eye(q.d0)*3.0*(qRef-q)+eye(q.d0)*1.0*(qDotRef-qDot));
      //u = TSharp*(yDDotDes + T*F) +  M*(eye(q.d0)*3.0*(qRef-q)+eye(q.d0)*1.0*(qDotRef-qDot));
      u = M*(inverse(A)*(~M*H*(M*(eye(q.d0)*0.0*(qRef-q)+eye(q.d0)*0.0*(qDotRef-qDot))+F-F)+~J*C*(Kp*(x0-x)+k)) + Kd*(qDotRef-qDot))+F;

      if(i > 30 && i < 40) {
        arr force = ARR(-0.0,0.0,10.0,0.0,0.0,0.0);
        //u += ~J*force;
        cout << "force" << endl;
      }

      world.stepDynamics(u, tau, 0.0);
      world.getJointState(q,qDot);
      q(world.getJointByName("l_gripper_l_finger_joint")->qIndex) = 0.0;
      world.setJointState(q,qDot);
      world.watch(false);
    }
  }

  world.watch(true);
}









void operationalControl() {

  ors::KinematicWorld world("model.kvg");

  MotionProblem MP(world);

  makeConvexHulls(MP.world.shapes);
  MP.world.gl().add(changeAlpha);
  MP.world.watch(true);

  arr q;
  arr qDot;
  MP.world.getJointState(q, qDot);


  uint contactT = MP.T/2;
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(MP.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("dircetion", new DefaultTaskMap(vecDiffTMT, MP.world, "endeffL", ors::Vector(1.0,0.0,0.0), "startShape", ors::Vector(0.0,0.0,-1.0)));
  t->setCostSpecs(contactT-2, MP.T, {0.0}, 10.0);

  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"startShape", NoVector));
  t->setCostSpecs(contactT-2, contactT, {0.0}, 1000.0);
  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"endShape", NoVector));
  t->setCostSpecs(MP.T-2, MP.T, {0.0}, 1000.0);

  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MF(MP);

  arr traj = MP.getInitialization();

  optConstrainedMix(traj, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  MP.costReport();

  arr qRef,qDotRef;
  qDotRef = zeros(q.d0);
  arr M,F,J,JAngle,xRef,x,xAngle,xAngleRef;
  arr u;
  arr KpTilde,KdTilde;

  arr Kp = zeros(7,7);
  Kp(0,0) = 100.0;
  Kp(1,1) = 100.0;
  Kp(2,2) = 100.0;
  Kp(3,3) = 100.0;
  Kp(4,4) = 100.0;
  Kp(5,5) = 100.0;
  Kp(6,6) = 100.0;
  arr Kd = eye(q.d0)*100.0;

  double tau = 0.01;

  arr qAccRef = zeros(q.d0);

  arr T,THash;
  arr C = eye(3)*1000.0;
  arr H = eye(q.d0)*10000.0;

  for(uint i = 1; i < traj.d0-1; i++) {
    mlr::wait(0.01);
    //MP.world.setJointState(x[i]);
    qRef = traj[i];

    qAccRef = (traj[i-1] + traj[i+1] - 2.0*traj[i])/MP.tau/MP.tau;
    cout << qAccRef <<endl;
    for(uint t = 0; t < 50; t++) {
      //mlr::wait(0.01);

      MP.world.equationOfMotion(M,F);
      MP.world.kinematicsPos(x, J, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.kinematicsQuat(xAngle, JAngle, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.getJointState(q, qDot);

      MP.world.setJointState(qRef,qDotRef);
      MP.world.kinematicsPos(xRef, NoArr, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.kinematicsQuat(xAngleRef, NoArr, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.setJointState(q,qDot);

     T = makeJacobian(J,JAngle)*inverse(M);
      //THash = inverse(~T*C*T + H) * ~T*C;
      //THash = inverse(H)*~T*inverse(T*H*~T);
       //~makeJacobian(J,JAngle)*Kp*(makeVec(xRef,xAngleRef)-makeVec(x,xAngle))<<endl<<endl;
      //u = THash*(Kp*(makeVec(xRef,xAngleRef)-makeVec(x,xAngle)) + Kd*J*(qDotRef-qDot) + T*F);
      u = M*(~makeJacobian(J,JAngle)*Kp*(makeVec(xRef,xAngleRef)-makeVec(x,xAngle))+Kd*(qDotRef-qDot))+F;
      //cout << Kp*(makeVec(xRef,xAngleRef)-makeVec(x,xAngle)) << endl;
      //u = zeros(q.d0);
      if(i > 30 && i < 40) {
      //  arr force = ARR(-0.0,0.0,0.0);
      //  u += ~J*force;
        cout << "force" << endl;
      }

      MP.world.stepDynamics(u, tau, 0.0);
      MP.world.watch(false);
    }
  }
  MP.world.watch(true);

}


void generateTrajectory() {

  ors::KinematicWorld world("model.kvg");

  MotionProblem MP(world);

  makeConvexHulls(MP.world.shapes);
  MP.world.gl().add(changeAlpha);
  MP.world.watch(true);

  arr q;
  arr qDot;
  MP.world.getJointState(q, qDot);


  uint contactT = MP.T/2;
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(MP.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("dircetion", new DefaultTaskMap(vecDiffTMT, MP.world, "endeffL", ors::Vector(1.0,0.0,0.0), "startShape", ors::Vector(0.0,0.0,-1.0)));
  t->setCostSpecs(contactT-2, MP.T, {0.0}, 10.0);

  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"startShape", NoVector));
  t->setCostSpecs(contactT-2, contactT, {0.0}, 1000.0);
  t = MP.addTask("bla", new DefaultTaskMap(posDiffTMT, MP.world, "endeffL", NoVector,"endShape", NoVector));
  t->setCostSpecs(MP.T-2, MP.T, {0.0}, 1000.0);

  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 1.0);
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MF(MP);

  arr traj = MP.getInitialization();

  optConstrainedMix(traj, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  MP.costReport();

  arr qRef,qDotRef;
  qDotRef = zeros(q.d0);
  arr M,F,J,JAngle,xRef,x,xAngle,xAngleRef;
  arr u;
  arr KpTilde,KdTilde;

  arr Kp = zeros(3,3);
  Kp(0,0) = 100.0;
  Kp(1,1) = 100.0;
  Kp(2,2) = 100.0;
  arr Kd = eye(3)*100.0;

  double tau = 0.01;

  arr qAccRef = zeros(q.d0);


  for(uint i = 1; i < traj.d0-1; i++) {
    mlr::wait(0.01);
    //MP.world.setJointState(x[i]);
    qRef = traj[i];
   // if(i = 20) {
   //   qRef = traj[20];
   // }
    qAccRef = (traj[i-1] + traj[i+1] - 2.0*traj[i])/MP.tau/MP.tau;
    cout << qAccRef <<endl;
    for(uint t = 0; t < 50; t++) {
      //mlr::wait(0.01);

      MP.world.equationOfMotion(M,F);
      MP.world.kinematicsPos(x, J, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.kinematicsQuat(xAngle, JAngle, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.getJointState(q, qDot);

      MP.world.setJointState(qRef,qDotRef);
      MP.world.kinematicsPos(xRef, NoArr, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.kinematicsQuat(xAngleRef, NoArr, MP.world.getBodyByName("l_gripper_l_finger_link"));
      MP.world.setJointState(q,qDot);

      KpTilde = ~J*Kp;
      KdTilde = Kd; //~J*Kd*J;

      //u = M * (~J*Kp*(xRef-x) + ~JAngle*eye(4)*50.0*(xAngleRef-xAngle) + (~J*Kd*J + ~JAngle*eye(4)*100.0*JAngle )*(qDotRef-qDot)) + F;
      //u = M * ((~J*Kp*J+~JAngle*Kp*JAngle) * (qRef-q) + KdTilde * (qDotRef-qDot)) + F;
      //u = M * (eye(q.d0)*100.0*(qRef-q)+ eye(q.d0)*50.0*(qDotRef-qDot))+F;

      //u = M*(~J*J*100.0*(qRef-q) + ~JAngle*JAngle*100.0*(qRef-q) + eye(q.d0)*100.0*(qDotRef-qDot)) + F;

      u = M*(eye(q.d0)*30.0*(qRef-q) + eye(q.d0)*100.0*(qDotRef-qDot)) + F;
      u +=  M*(~J*Kp*(xRef-x) + ~JAngle*eye(4)*50.0*(xAngleRef-xAngle));

      if(i > 30 && i < 40) {
        arr force = ARR(-100.0,0.0,0.0);
        u += ~J*force;
        cout << "force" << endl;
      }

      MP.world.stepDynamics(u, tau, 0.0);
      MP.world.watch(false);
    }
  }
  /*for(uint t = 0; t < 10000; t++) {
    mlr::wait(0.001);

    MP.world.equationOfMotion(M,F);
    MP.world.kinematicsPos(x, J, MP.world.getBodyByName("l_gripper_l_finger_link"));
    MP.world.kinematicsQuat(xAngle, JAngle, MP.world.getBodyByName("l_gripper_l_finger_link"));
    MP.world.getJointState(q, qDot);

    MP.world.setJointState(qRef,qDotRef);
    MP.world.kinematicsPos(xRef, NoArr, MP.world.getBodyByName("l_gripper_l_finger_link"));
    MP.world.kinematicsQuat(xAngleRef, NoArr, MP.world.getBodyByName("l_gripper_l_finger_link"));
    MP.world.setJointState(q,qDot);

    KpTilde = ~J*Kp;
    KdTilde = Kd; //~J*Kd*J;

    u = M * (~J*Kp*(xRef-x) + ~JAngle*eye(4)*20.0*(xAngleRef-xAngle) + (~J*Kd*J + ~JAngle*eye(4)*10.0*JAngle )*(qDotRef-qDot)) + F;
    //u = M * ((~J*Kp*J+~JAngle*Kp*JAngle) * (qRef-q) + KdTilde * (qDotRef-qDot)) + F;
    //u = M * (eye(q.d0)*100.0*(qRef-q)+ eye(q.d0)*50.0*(qDotRef-qDot))+F;

    MP.world.stepDynamics(u, tau, 0.0);
    MP.world.watch(false);
  }*/
  MP.world.watch(true);

}

void control4() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);
  world.watch(true);

  arr q;
  arr qDot;
  world.getJointState(q, qDot);

  double tau = 0.01;

  arr u = zeros(q.d0,1);
  //arr u = qDot;
  cout << world.getJointStateDimension() << endl;

  arr Kp = zeros(3,3);
  Kp(0,0) = 1000.0;
  Kp(1,1) = 1000.0;
  Kp(2,2) = 1000.0;

  arr KpAngle = zeros(3,3);
  KpAngle(0,0) = 100.0;
  KpAngle(1,1) = 100.0;
  KpAngle(2,2) = 100.0;

  arr Kd;
  //Kd = eye(3)*80.0;
  Kd = eye(q.d0)*50.0;

  arr qRef = zeros(q.d0);//ARR(0.0,0.0,0.0,0.0,3.0/2,3.0,0.0,0.0,0.0,0.0,0.0,0.0,3.0/2,3.0,0.0,0.0,0.0);
  qRef(5) = 3.0/2.0;
  arr qDotRef = zeros(q.d0);//ARR(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);

  arr M,F,J,JAngle;

  arr KpTilde,KdTilde;

  arr x,xRef,xDotRef;

  arr force = ARR(0.0,0.0,1000.0);

  for(uint t = 0; t < 10000; t++) {
    mlr::wait(0.01);

    world.equationOfMotion(M,F);
    world.kinematicsPos(x, J, world.getBodyByName("l_wrist_roll_link"));
    world.kinematicsVec(NoArr, JAngle, world.getBodyByName("l_wrist_roll_link"));
    world.getJointState(q, qDot);

    world.setJointState(qRef,qDotRef);
    world.kinematicsPos(xRef, NoArr, world.getBodyByName("l_wrist_roll_link"));
    world.setJointState(q,qDot);

    KpTilde = ~J*Kp;//+ ~JAngle*KpAngle*JAngle;
    KdTilde = Kd; //~J*Kd*J;

    //u = M * (KpTilde * (xRef-x) + KdTilde * (qDotRef-qDot)) + F;

    u = M * (eye(q.d0)*100.0*(qRef-q)+ Kd*(qDotRef-qDot))+F;
    u(0) = 0.0; //fix torso lift

    if(t > 300 && t < 400) {
      u += ~J*force;
      cout << "force" << endl;
    }

    world.stepDynamics(u, tau, 0.0);
    world.watch(false);
  }
}

void control3() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);
  world.watch(true);

  arr q;
  arr qDot;
  world.getJointState(q, qDot);

  double tau = 0.01;

  arr u = zeros(q.d0,1);
  //arr u = qDot;
  cout << world.getJointStateDimension() << endl;

  arr Kp = zeros(6,6);
  Kp(0,0) = 100.0;
  Kp(1,1) = 100.0;
  Kp(2,2) = 100.0;
  Kp(3,3) = 100.0;
  Kp(4,4) = 100.0;
  Kp(5,5) = 100.0;

  arr Kd;
  //Kd = eye(3)*80.0;
  Kd = eye(3)*100.0;

  arr KdAngle;
  KdAngle = eye(3)*50.0;

  arr qRef = ARR(0.0,0.0,0.0,3.0/2,3.0,0.0,0.0);
  arr qDotRef = ARR(0.0,0.0,0.0,0.0,0.0,0.0,0.0);

  arr M,F,J,JAngle;

  arr KpTilde,KdTilde;

  arr force = ARR(100.0,100.0,100.0);

  for(uint t = 0; t < 10000; t++) {
    mlr::wait(0.01);

    world.equationOfMotion(M,F);
    world.kinematicsPos(NoArr, J, world.getBodyByName("l_wrist_flex_link"));
    world.jacobianR(JAngle, world.getBodyByName("l_wrist_flex_link"));
    arr JPos = zeros(6,6);
    JPos(0,0) = J(0,0);
    JPos(0,1) = J(0,1);
    JPos(0,2) = J(0,2);
    JPos(1,0) = J(1,0);
    JPos(1,1) = J(1,1);
    JPos(1,2) = J(1,2);
    JPos(2,0) = J(2,0);
    JPos(2,1) = J(2,1);
    JPos(2,2) = J(2,2);
    JPos(3,3) = JAngle(0,0);
    JPos(3,4) = JAngle(0,1);
    JPos(3,5) = JAngle(0,2);
    JPos(4,3) = JAngle(1,0);
    JPos(4,4) = JAngle(1,1);
    JPos(4,5) = JAngle(1,2);
    JPos(5,3) = JAngle(2,0);
    JPos(5,4) = JAngle(2,1);
    JPos(5,5) = JAngle(2,2);
    KpTilde = inverse(eye(q.d0)/100.0+~J*J)*~J*Kp*J;
    KdTilde = eye(q.d0)*100.0;
    u = M*(KpTilde * (qRef-q) + KdTilde * (qDotRef-qDot)) + F;


    if(t > 300 && t < 400) {
      u += ~J*force;
      cout << "force" << endl;
    }

    world.stepDynamics(u, tau, 0.0);
    world.watch(false);
    world.getJointState(q, qDot);
  }
}



void control2() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);
  world.watch(true);

  arr q;
  arr qDot;
  world.getJointState(q, qDot);

  double tau = 0.01;

  arr u = zeros(q.d0,1);
  //arr u = qDot;
  cout << world.getJointStateDimension() << endl;

  arr Kp = zeros(3,3);
  Kp(0,0) = 50.0;
  Kp(1,1) = 50.0;
  Kp(2,2) = 1000.0;

  arr KpAngle = zeros(3,3);
  KpAngle(0,0) = 50.0;
  KpAngle(1,1) = 1.0;
  KpAngle(2,2) = 50.0;

  arr Kd;
  //Kd = eye(3)*80.0;
  Kd = eye(q.d0)*10.0;

  arr qRef = ARR(0.0,0.0,0.0,3.0/2,3.0,0.0,0.0);
  arr qDotRef = ARR(0.0,0.0,0.0,0.0,0.0,0.0,0.0);

  arr M,F,J,JAngle;

  arr KpTilde,KdTilde;

  arr force = ARR(100.0,100.0,100.0);

  for(uint t = 0; t < 10000; t++) {
    mlr::wait(0.01);

    world.equationOfMotion(M,F);
    world.kinematicsPos(NoArr, J, world.getBodyByName("l_wrist_roll_link"));
    world.jacobianR(JAngle, world.getBodyByName("l_wrist_roll_link"));
    KpTilde = ~J*Kp*J + ~JAngle*KpAngle*JAngle;
    KdTilde = Kd; //~J*Kd*J;
    //KpTilde = eye(7)*10.0;
    u = M*(eye(7)*10.0*(qRef-q)+KdTilde * (qDotRef-qDot)) + (KpTilde * (qRef-q)) + F;


    if(t > 300 && t < 400) {
      u += ~J*force;
      cout << "force" << endl;
    }

    world.stepDynamics(u, tau, 0.0);
    world.watch(false);
    world.getJointState(q, qDot);
  }
}

void control() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world.gl().add(changeAlpha);
  world.watch(true);

  arr q;
  arr qDot;
  world.getJointState(q, qDot);

  double tau = 0.01;

  arr u = zeros(q.d0,1);
  //arr u = qDot;
  cout << world.getJointStateDimension() << endl;

  arr Kp = zeros(3,3);
  Kp(0,0) = 100.0;
  Kp(1,1) = 10000.0;
  Kp(2,2) = 100.0;

  arr KpAngle = zeros(3,3);
  KpAngle(0,0) = 100.0;
  KpAngle(1,1) = 100.0;
  KpAngle(2,2) = 100.0;

  arr Kd;
  //Kd = eye(3)*80.0;
  Kd = eye(q.d0)*50.0;

  arr qRef = ARR(0.0,0.0,0.0,3.0/2,3.0,0.0,0.0);
  arr qDotRef = ARR(0.0,0.0,0.0,0.0,0.0,0.0,0.0);

  arr M,F,J,JAngle;

  arr KpTilde,KdTilde;

  arr force = ARR(0.0,1000.0,0.0);

  for(uint t = 0; t < 10000; t++) {
    mlr::wait(0.01);

    world.equationOfMotion(M,F);
    world.kinematicsPos(NoArr, J, world.getBodyByName("l_wrist_roll_link"));
    world.kinematicsVec(NoArr, JAngle, world.getBodyByName("l_wrist_roll_link"));
    //cout << ~JAngle << endl << endl;
    KpTilde = ~J*Kp*J ;//+ ~JAngle*KpAngle*JAngle;
    KdTilde = Kd; //~J*Kd*J;
    //cout << (qRef-q) << endl;
    //cout << KpTilde << endl;
    //cout << M << endl << endl;
    u = M * ((eye(q.d0)*50.0 + KpTilde) * (qRef-q) + KdTilde * (qDotRef-qDot)) + F;
    //u = M * (KpTilde * (qRef-q) + KdTilde * (qDotRef-qDot)) + F;


    if(t > 300 && t < 400) {
      u += ~J*force;
      cout << "force" << endl;
    }

    world.stepDynamics(u, tau, 0.0);
    world.watch(false);
    world.getJointState(q, qDot);
  }


  /*
  TrajectoryInterface *ti = new TrajectoryInterface(world);
  ti->world->gl().update();
  ti->world->gl().resize(800,800); //segmentation fault again
  ti->world->gl().add(changeColor2);

  arr q = ti->world->getJointState();

  MotionProblem MP(world);

  Task *t;

  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  double contactT = MP.T/2;

  t = MP.addTask("box_wheel_plate_joint", new TaskMap_qItself(world.getJointByName("box_wheel_plate_joint")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(MP.T-20, MP.T, {-3.14/2}, 1e3);

  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC1",NoVector, "knob_cp2",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 10.);
  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC2",NoVector, "knob_cp1",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 10.);

  t = MP.addTask("fix_joint", new qItselfConstraint(world.getJointByName("box_wheel_plate_joint")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(0.,contactT, {0.}, 1.);

  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //ShapeL except = world.getBodyByName("l_wrist_roll_link")->shapes;
  //t = MP.addTask("collision", new ProxyConstraint(allExceptListedPTMT, shapesToShapeIndices(except), 0.1));
  //t->setCostSpecs(0., MP.T, {0.}, 1.);
  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 100.);

  //t = MP.addTask("direction", new VelAlignConstraint(world, "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291), "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291)));
  //t = MP.addTask("direction", new VelAlignConstraint(world, "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291), "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291)));
  //t = MP.addTask("direction", new DefaultTaskMap(world, "endeffL", ors::Vector(1,0,0), "ve_shape", NoVector));
  //t = MP.addTask("direction", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(1,0,0), "ve_shape", ors::Vector(1,0,0)));
  //t->setCostSpecs(contactT-10, contactT, {0.9}, 10.);

  MotionProblemFunction MF(MP);

  arr x = MP.getInitialization();

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  MP.costReport();
  ti->executeTrajectory(x, 10);

  /// get contact point trajectory
  arr Y;
  for (uint i=contactT;i<x.d0;i++){
    world.setJointState(x[i]);
    arr tmp = conv_vec2arr(world.getShapeByName("endeffC1")->X.pos);
    Y.append(~tmp);
  }

  /// detect DOF
  ors::Transformation T;
  detectDOF(Y,T);
  ti->world->getBodyByName("dof_axis")->X = T;
  ti->world->calc_fwdPropagateFrames();
  ti->world->watch(true);
  cout << T.rot << endl;
  ti->~TrajectoryInterface();
  makeNewWorld(T);

  //world.getBodyByName("dof_axis")->X = T;
  //world.calc_fwdPropagateFrames();

  //world.watch(true);

  //q(ti->world->getJointByName("l_gripper_l_finger_joint")->qIndex)+= 0.2;

  //ti->~TrajectoryInterface();
  //ti->executeTrajectory(x, 10);
  //ti->executeTrajectory(x, 10);
  //ti->executeTrajectory(x, 10);
  //ti->executeTrajectory(x, 10);
*/
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  //control4();
  //generateTrajectory();
  //operationalControl();
  //displayTrajectory();
  //controlJointSpace();
  //controlStiffnessSpace();
  controlOperationalSpace();
  //exTrajectoryOnPr2();
  return 0;
}
