#include "traj_optimizer.h"


void TrajOptimizer::optimizeTrajectory() {
    // Plan Trajectory
    MotionProblem MP(world);
    MP.loadTransitionParameters();

    arr refGoal;
    refGoal = ARRAY(MP.world.getBodyByName("goalRef")->X.pos);
    refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);
//    if (useGoalPub) {
//      refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);
//      // Get Reference Goal
//      get_goal_client.call(goalSub);
//      refGoal = refFrame + ARRAY(goalSub.response.x,goalSub.response.y,goalSub.response.z);
//      world.getBodyByName("goalRef")->X.pos = refGoal;
//    } else{

//    }

    cout << "refGoal: "<<refGoal << endl;

    //-- create an optimal trajectory to goalRef
    arr qLimits = { -224, 44,   //17 r_upper_arm_roll_joint
                    -130, 40,   //18 r_shoulder_pan_joint
                    -30, 80,    //19 r_shoulder_lift_joint
                    -360, 360,  //20 r_forearm_roll_joint
                    -133, 0,    //21 r_elbow_flex_joint
                    -130, 0,    //22 r_wrist_flex_joint
                    -360, 360}; //23 r_wrist_roll_joint
    qLimits = M_PI*qLimits/180.;
    qLimits.reshape(7,2);

    //-- create an optimal trajectory to trainTarget
    TaskCost *c;
    c = MP.addTaskMap("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
    MP.setInterpolatingCosts(c, MotionProblem::finalOnly,
                             refGoal, 1e1,
                             ARRAY(0.,0.,0.), 1e-4);
    MP.setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                                ARRAY(0.,0.,0.), 1e1,
                                ARRAY(0.,0.,0.), 0.);

    c = MP.addTaskMap("qlimits", new DefaultTaskMap(qLimitsTMT,world, NULL, NoVector, NULL, NoVector, qLimits));
    MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e2);

    c = MP.addTaskMap("homing", new DefaultTaskMap(qItselfTMT,world, NULL, NoVector, NULL, NoVector));
    MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e-3);

    q0 = {-10.,-20.,20.,0.,-20.,-20.,0.};
    //    q0 = M_PI*q0/180.;
    //    cout <<"q0 "<< q0 << endl;
    my_controller_pkg::GetState getStateSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      rt_get_state_clients[i].call(getStateSrv);
      q0(i) = getStateSrv.response.pos;
    }
    cout << q0 << endl;

    //-- create the Optimization problem (of type kOrderMarkov)
    MP.x0 = q0;

    MotionProblemFunction MPF(MP);
    uint T=MPF.get_T();
    uint k=MPF.get_k();
    uint n=MPF.dim_x();
    double dt = MP.tau;
    cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

    arr x(T+1,n); x.setZero();
    optNewton(x, Convert(MPF), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
    MP.costReport(false);
    displayTrajectory(x, 1, world, "planned trajectory", 0.0001);


    //-- Transform trajectory into task space
    arr kinPos, kinVec, xRefPos, xRefVec;
    // store cartesian coordinates and endeffector orientation
    for (uint t=0;t<=T;t++) {
      world.setJointState(x[t]);
      world.calcBodyFramesFromJoints();
      world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeff")->index);
      world.kinematicsVec(kinVec,NoArr,MP.world.getBodyByName("endeff")->index);
      xRefPos.append(~kinPos);
      xRefVec.append(~kinVec);
    }

    tauRef = ~cat(~xRefPos,~xRefVec);

    // set initial positions
    q0 = x[0]; arr dq0 = 0.*q0;
    q = q0; qd = dq0;
    x0 = tauRef[0];
    world.setJointState(q,qd);
    world.calcBodyFramesFromJoints();
    TRef = T*dt;
  }
