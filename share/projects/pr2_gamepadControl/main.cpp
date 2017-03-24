#include <Control/gamepad2tasks.h>
#include <Control/taskControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Gui/opengl.h>

#include <RosCom/roscom.h>
#include <RosCom/rosmacro.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



void changeColor(void*){  orsDrawAlpha = .5; glColor(.5,.0,.0); }
void changeColor2(void*){  orsDrawAlpha = 1.; }

void TEST(Gamepad){
  bool useRos = mlr::getParameter<bool>("useRos", false);
  bool fixBase = mlr::getParameter<bool>("fixBase", false);

  //-- setup the system (modules and accesses)
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(arr, gamepadState)
  ACCESSname(arr, pr2_odom)

  GamepadInterface mod_gamepad;
  RosCom_Spinner mod_rosSpinner;
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub_ctrl_obs("/marc_rt_controller/jointState", ctrl_obs);
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub_ctrl_ref("/marc_rt_controller/jointReference", ctrl_ref);
  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub_pr2_odom("/robot_pose_ekf/odom_combined", pr2_odom);
//  RosCom_ForceSensorSync(); //NULL, Module::loopWithBeat, 1.);

  //-- open modules
  threadOpenModules(true);

  mlr::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  mlr::Joint *trans=world.getJointByName("worldTranslationRotation");

  mlr::KinematicWorld world_pr2 = world;
  world.gl().add(changeColor);
  world.gl().addDrawer(&world_pr2);
  world.gl().add(changeColor2);

  TaskControlMethods MP(world, true);
  MP.qitselfPD.y_ref = q;
  MP.H_rate_diag = world.getHmetric();
  Gamepad2Tasks j2t(MP);
  MP.tasks = j2t.getTasks();

  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    uint trials=0;
    for(;useRos;){
      ctrl_obs.data->waitForNextRevision();
      cout <<"REMOTE joint dimension=" <<ctrl_obs.get()->q.N <<endl;
      cout <<"LOCAL  joint dimension=" <<MP.world.q.N <<endl;

      if(ctrl_obs.get()->q.N==MP.world.q.N
         && ctrl_obs.get()->qdot.N==MP.world.q.N)
        break;

      trials++;
      if(trials>20){
        HALT("sync'ing real PR2 with simulated failed - using useRos=false")
      }
    }

    //-- wait for first odometry observation!
    for(trials=0;useRos;){
      pr2_odom.data->waitForNextRevision();
      if(pr2_odom.get()->N==3) break;

      trials++;
      if(trials>20){
        HALT("sync'ing real PR2 odometry failed")
      }
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = ctrl_obs.get()->q;
    qdot = ctrl_obs.get()->qdot;
    q({trans->qIndex, trans->qIndex+2}) = pr2_odom.get();
    //arr fL_base = fL_obs.get();
    MP.setState(q, qdot);
  }
  arr zero_qdot(qdot.N);
  zero_qdot.setZero();
  CtrlMsg refs;

  arr fLobs,uobs;
  ofstream fil("z.forces");
  mlr::arrayBrackets="  ";

  for(uint t=0;;t++){
    gamepadState.data->waitForNextRevision();
    arr gamepad = gamepadState.get();
    bool gamepad_shutdown = j2t.updateTasks(gamepad);
//    if(t>10 && gamepad_shutdown) moduleShutdown()->incrementValue();


    //get joint state
    if(useRos){
      arr q_real =ctrl_obs.get()->q;
      q_real({trans->qIndex, trans->qIndex+2}) = pr2_odom.get();

      world_pr2.setJointState(q_real, ctrl_obs.get()->qdot);
      fLobs = ctrl_obs.get()->fL;
      uobs = ctrl_obs.get()->u_bias;
    }

    //compute control
    arr a = MP.operationalSpaceControl();
    q += .01*qdot;
    qdot += .01*a;
    MP.reportCurrentState();
    MP.setState(q, qdot);
    //MP.world.reportProxies();
    if(!(t%4))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    //-- force task
    uint mode = 0;
    if(gamepad.N) mode = uint(gamepad(0));
    if(mode==2){
      cout <<"FORCE TASK" <<endl;
#if 0 // set a feew fwd force task
      refs.fL = ARR(10., 0., 0.);
      refs.u_bias = 1.*(~J_fL * refs.fL);
      refs.Kp = ARR(.3);
#else // apply force in direction fe
      arr f_des = ARR(0.,0.,-5.);
      double alpha = .003;
      mlr::Shape *ftL_shape = world.getShapeByName("endeffForceL");
      arr J_ft, J;
      MP.world.kinematicsPos(NoArr, J, ftL_shape->body, ftL_shape->rel.pos);
      MP.world.kinematicsPos_wrtFrame(NoArr, J_ft, ftL_shape->body, ftL_shape->rel.pos, MP.world.getShapeByName("l_ft_sensor"));

      refs.u_bias = ~J*f_des;
      refs.fL = f_des;
      refs.KiFTL = alpha*~J;
      refs.J_ft_invL = inverse_SymPosDef(J_ft*~J_ft)*J_ft;

      J = inverse_SymPosDef(J*~J)*J;

      fil <<t <<' ' <<f_des <<' ' << J_ft*fLobs << " " << J*uobs << endl;


//      // change stiffness of endeff
//      -      arr y_fL, J_fL;
//      -      MP.world.kinematicsPos(y_fL, J_fL, ftL_shape->body, &ftL_shape->rel.pos);
//      -      J_fL = J_fL.sub(0,1,0,-1);
//      -      arr gain = 10.*(~J_fL*J_fL) + .3*eye(q.N);
//      -      cout <<J_fL <<gain <<endl;
//      -      refs.Kq_gainFactor = gain;
//      -      refs.u_bias = zeros(q.N);

      // compute position gains that are 0 along force direction
//      arr yVec_fL, JVec_fL;
//      mlr::Vector rel = ftL_shape->rel.rot*mlr::Vector(fe/length(fe));
//      MP.world.kinematicsVec(yVec_fL, JVec_fL, ftL_shape->body,&rel);

//      mlr::Quaternion quat;
//      quat.setDiff(mlr::Vector(1.,0.,0.),yVec_fL);
//      arr R = ~quat.getArr();
//      arr J_fL0 = R*Jeq;
//      J_fL0[0]=0.;

//      refs.Kp = ~Jeq*inverse(Jeq * ~Jeq)*J_fL0;
//      refs.Kp = 10.*refs.Kp + 0.1*eye(q.N);

//       test gains
//      arr dq;
//      MP.world.getJointState(dq);
//      dq = dq*0.+0.1;
//      cout << refs.Kp*dq << endl;
//      cout << ~R*J_fL0*dq << endl;
//      arr dy = ~R*Jeq*refs.Kp*dq;
//      cout << R*dy << endl;

#endif
    }else{
      refs.fL = zeros(6);
      refs.KiFTL.clear();
      refs.J_ft_invL.clear();
      refs.u_bias = zeros(q.N);
    }
    refs.Kp = 1.;
    if(mode==2) refs.Kp = .1;
    refs.Kd = 1.;
    refs.Ki = 0.;
    refs.fL_gamma = 1.;

    refs.q=q;
    refs.qdot=zero_qdot;
    if(!fixBase && trans && trans->qDim()==3){
      refs.qdot(trans->qIndex+0) = qdot(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot(trans->qIndex+2);
    }
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 0.3;

    if(useRos){
      ctrl_ref.set() = refs;
    }

    if(moduleShutdown()->getStatus()/* || !rosOk()*/) break;
  }

  threadCloseModules();
  threadReportCycleTimes();
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  rosCheckInit("pr2_gamepad");
  testGamepad();
  return 0;
}
