#include "pd_executor_module.h"
#include <Ors/ors.h>
#include <Motion/pr2_heuristics.h>


//#ifdef WITH_ROS
  //  #include <pr2/roscom.h>
//#endif

// ############################################################################
// Executor
PDExecutor::PDExecutor()
    : world("model.kvg"),worldreal("model.kvg"), fmc(world, true), useros(false),
      limits(nullptr), collision(nullptr),
      effPosR(nullptr), gripperR(nullptr), effOrientationR(nullptr),
      effPosL(nullptr), gripperL(nullptr), effOrientationL(nullptr)
{
  // fmc setup
  world.getJointState(q, qdot);
  fmc.H_rate_diag = pr2_reasonable_W(world);
  fmc.qitselfPD.y_ref = q;
  fmc.qitselfPD.setGains(2.3,10);
//  fmc.qitselfPD.active = false;
  fmc.qitselfPD.prec = 0.100;
  //fmc.qitselfPD.maxVel = 0.008;
  //fmc.qitselfPD.maxAcc = 0.09;
  //fmc.qitseldPD.f_Igain = 1.;

  if(MT::getParameter<bool>("useLimits", false)) {
    limits = fmc.addPDTask("limits", 0.2, .8, new TaskMap_qLimits);
   // limits->y_ref.setZero();

  limits->prec = 0.1;
  }

  if(MT::getParameter<bool>("useCollisions", false)) {
    collision = fmc.addPDTask("collision", 0.1, 5.8, new ProxyTaskMap(allPTMT, {0u}, .1));
  }

  if(MT::getParameter<bool>("usePositionR", false)) {

    effPosR = fmc.addPDTask("MoveEffTo_endeffR", .2, 1.8,new DefaultTaskMap(posTMT,world,"endeffR",NoVector,"base_footprint"));
    effPosR->y_ref = {0.8, -.5, 1.};
    //effPosR->maxVel = 0.004;
  }

  if(MT::getParameter<bool>("usePositionL", false)) {
    effPosL = fmc.addPDTask("MoveEffTo_endeffL", .2, 1.8,new DefaultTaskMap(posTMT,world,"endeffL",NoVector,"base_footprint"));
    effPosL->y_ref = {0.8, .5, 1.};
    //effPosL->f_ref = {5.,5.,5.};
    //effPosL->f_Igain = .05;
    //effPosL->maxVel = 0.004;
  }
  if(MT::getParameter<bool>("fc", false)) {
    fc = fmc.addPDTask("fc_endeffL", .2, 1.8,new DefaultTaskMap(posTMT,world, "endeffForceL",NoVector,"base_footprint"));
    fc->y_ref ={0.8,0.5,1.}; 
    fc->f_ref = {5.,5.,5.};
    fc->f_Igain = .08;
  }

  if(MT::getParameter<bool>("useGripperR", false)) {
    int jointID = world.getJointByName("r_gripper_joint")->qIndex;
    gripperR = fmc.addPDTask("gripperR", .3, 1.8, new TaskMap_qItself(jointID, world.q.N));
    gripperR->setTarget({0.01});
    //gripperR->y_ref = {.08};  // open gripper 8cm
  }

  if(MT::getParameter<bool>("useGripperL", false)) {
    int jointID = world.getJointByName("l_gripper_joint")->qIndex;
    gripperL = fmc.addPDTask("gripperL", .3, 1.8, new TaskMap_qItself(jointID, world.q.N));
    gripperL->setTarget({0.01});
    //gripperL->y_ref = {.08};  // open gripper 8cm
  }

  if(MT::getParameter<bool>("useOrientationR", false)) {
    effOrientationR = fmc.addPDTask("orientationR", .2, 1.8,new DefaultTaskMap(quatTMT,world, "endeffR"));
    effOrientationR->y_ref = {1., 0., 0., 0.};
    effOrientationR->flipTargetSignOnNegScalarProduct = true;

  }

  if(MT::getParameter<bool>("useOrientationL", false)) {
    effOrientationL = fmc.addPDTask("orientationL", .2,1.8,new DefaultTaskMap(quatTMT,world, "endeffL"));
    effOrientationL->y_ref = {1., 0., 0., 0.};
    effOrientationL->flipTargetSignOnNegScalarProduct = true;

  }


  if(MT::getParameter<bool>("base", false)) {
    base = fmc.addPDTask("basepos", .2,.8,new TaskMap_qItself(world, "worldTranslationRotation"));
  }

}

void PDExecutor::visualizeSensors()
{
  floatA rh = poses_rh.get();
  if(rh.N) {
    world.getShapeByName("sensor_rh_thumb")->rel.pos = ors::Vector(rh(0, 0), rh(0, 1), rh(0, 2));
    world.getShapeByName("sensor_rh_index")->rel.pos = ors::Vector(rh(1, 0), rh(1, 1), rh(1, 2));
  }
  floatA lh = poses_lh.get();
  if(lh.N) {
    world.getShapeByName("sensor_lh_thumb")->rel.pos = ors::Vector(lh(0, 0), lh(0, 1), lh(0, 2));
    world.getShapeByName("sensor_lh_index")->rel.pos = ors::Vector(lh(1, 0), lh(1, 1), lh(1, 2));
  }
}


void setOdom(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose){
  ors::Quaternion quat(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
  ors::Vector pos(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);

  double angle;
  ors::Vector rotvec;
  quat.getRad(angle, rotvec);
  q(qIndex+0) = pos(0);
  q(qIndex+1) = pos(1);
  q(qIndex+2) = MT::sign(rotvec(2)) * angle;
  cout<<q<<endl;
}



void PDExecutor::step()
{
    cout<<"\x1B[2J\x1B[H";
    if (useros && !inited)
    {
        cout << "STARTING TO OPEN" << endl;
        initRos();
        cout << "FINISHED TO OPEN" << endl;
        inited = true;
    }

    world.watch(false);
    worldreal.watch(false);
    
    ors::Joint *trans= world.getJointByName("worldTranslationRotation");
    arr fLobs;
    arr uobs;
    if(useros)
    {
       // ctrl_obs.waitForNextRevision();
       // pr2_odom.waitForRevisionGreaterThan(0);
        ors::Shape *ftL_shape = worldreal.getShapeByName("endeffForceL");
        CtrlMsg obs = ctrl_obs.get();
        fLobs = obs.fL;
        //cout<<fLobs<<endl;
        uobs =  obs.u_bias;
        //cout<<uobs<<endl;
        if(fLobs.N && uobs.N)
        {

            setOdom(obs.q,trans->qIndex,pr2_odom.get());

            worldreal.setJointState(obs.q,obs.qdot);
            arr Jft, J;
            worldreal.kinematicsPos(NoArr,J,ftL_shape->body,&ftL_shape->rel.pos);
            worldreal.kinematicsPos_wrtFrame(NoArr,Jft,ftL_shape->body,&ftL_shape->rel.pos,worldreal.getShapeByName("l_ft_sensor"));
            Jft = inverse_SymPosDef(Jft*~Jft)*Jft;
            J = inverse_SymPosDef(J*~J)*J;
             fLobs = Jft*fLobs;
             cout <<zeros(3) <<' ' << fLobs << " " << J*uobs << endl;
        }
    }     




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
        fc->active = false;
        base->active =false;
    }
    else
    {

       // effPosR->active = true;
      //  effPosL->active = true;
      //  effOrientationR->active = true;
      //  effOrientationL->active = true;
     //   gripperL->active = true;
     //   gripperR->active = true;
     //   fc->active = true;
        base->active =true;
    }


    if(!init)
    {
        // set arm poses
        double x, y, z;
        arr pos, quat;

        x = cal_pose_rh(0) * 1;
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


        x = cal_pose_lh(0) * 1;
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


        double cal_gripper;
        cal_gripper =  calibrated_gripper_rh.get();
        if(gripperR) gripperR->setTarget({cal_gripper});
        cal_gripper =  calibrated_gripper_lh.get();
        if(gripperL) gripperL->setTarget({cal_gripper});

        base->setTarget({1.,-2.,-6.28});
        fmc.qitselfPD.y_ref(trans->qIndex+0) = base->y_ref(trans->qIndex+0);
        fmc.qitselfPD.y_ref(trans->qIndex+1) = base->y_ref(trans->qIndex+1);
        fmc.qitselfPD.y_ref(trans->qIndex+2) = base->y_ref(trans->qIndex+2);
    }
    double tau = 0.001;

    for (uint t = 0; t < 20 ; t++)
    {
        arr a = fmc.operationalSpaceControl();
        q += tau * qdot;
        qdot += tau * a;
       

/*        if(fixBase.get())
        {
            qdot(trans->qIndex+0) = 0;
            qdot(trans->qIndex+1) = 0;
            qdot(trans->qIndex+2) = 0;
            q(trans->qIndex+0) = 0;
            q(trans->qIndex+1) = 0;
            q(trans->qIndex+2) = 0;
        }
        */
        fmc.setState(q, qdot);
  
    }
    cout<<    q(trans->qIndex+0)<<endl
        <<    q(trans->qIndex+1)<<endl
        <<    q(trans->qIndex+2)<<endl<<trans->qIndex<<endl;
    // set state
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
    ref.gamma = .988;
    ref.J_ft_inv.clear();
    fmc.reportCurrentState();
    //if (!fixBase.get() && trans && trans->qDim()==3)
   // {
     //   ref.qdot(trans->qIndex+0) = qdot(trans->qIndex+0);
     //   ref.qdot(trans->qIndex+1) = qdot(trans->qIndex+1);
     //   ref.qdot(trans->qIndex+2) = qdot(trans->qIndex+2);
  //  }
    if(useros)
    {
        uint count=0;
        //ctrlTasks.readAccess();
        //fm.tasks = ctrlTasks();
        if(fLobs.N && uobs.N)
        {

            for(CtrlTask *t : fmc.tasks)
            {
         
                if(t->active && t->f_ref.N)
                {
                    count++;
                    if(count!=1) HALT("you have multiple active force control tasks - NIY");
                    t->getForceControlCoeffs(ref.fL, ref.u_bias, ref.Ki, ref.J_ft_inv, worldreal);
                }
            }     
        }
        if(count==1 )
        {
            for(uint i = 0;i<fLobs.N;i++)
            {
                  if(fLobs(i) > fc->f_ref(i))  error(i) =ref.gamma*error(i)+ fc->f_ref(i)-fLobs(i);
                  else if(fLobs(i) < -fc->f_ref(i)) error(i) =ref.gamma*error(i)+ -fc->f_ref(i)-fLobs(i);
                  else error(i)=0.;
            }
        }    
        //  cout<<"fl"<<ref.fL<<endl<<"ubias"<<ref.u_bias<<endl<<"KI"<<ref.Ki<<endl<<"J_ft"<< ref.J_ft_inv<<endl;
        // ctrlTasks.deAccess();
    
        //-- send the computed movement to the robot
        cout<<"error"<<error<<endl;
        ctrl_ref.set() = ref;
    }
    
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
  ref.gamma = .5;
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
    ctrl_obs.var->waitForNextRevision();
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
  worldreal.setJointState(q,qdot);
  cout << "DONE" << endl;
//#endif
}

void PDExecutor::open()
{
  useros = MT::getParameter<bool>("useRos", false);
  error = zeros(3);
}

void PDExecutor::close()
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
    ref.gamma = 0.;
    ref.J_ft_inv.clear();
    //ctrl_ref.set() = ref;
}
