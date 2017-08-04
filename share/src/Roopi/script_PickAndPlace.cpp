#include "script_PickAndPlace.h"
#include "roopi.h"
#include <Control/taskControl.h>
#include <KOMO/komo.h>
#include <Kin/frame.h>

int Script_setGripper(Roopi& R, LeftOrRight lr, double gripSize){
  //query some info from the kinematics first
  uint grasp1, grasp2;
  {
    auto K = R.getK();
    if(R.getRobot()=="pr2"){
      if(lr==LR_right){
        grasp1 = K().getFrameByName("r_gripper_joint")->ID;
        grasp2 = K().getFrameByName("r_gripper_l_finger_joint")->ID;
      }else{
        grasp1 = K().getFrameByName("l_gripper_joint")->ID;
        grasp2 = K().getFrameByName("l_gripper_l_finger_joint")->ID;
      }
    }else{
      NIY;
    }
  }

  {
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    R.wait({-gripperR, -gripper2R});
  }
  return AS_done;
}


int Script_graspBox(Roopi& R, const char* objName, LeftOrRight rl){

  //query some info from the kinematics first
  double width, above;
  //uint obj, eff, grasp1, grasp2, workspace;
  uint obj, eff, grasp1, grasp2;
  {
    auto K = R.getK();

    //get obj size
    arr objSize = K().getFrameByName(objName)->shape->size;
    //width = objSize(1);
    width = objSize(0);
    above = .5*objSize(2);

    //relevant shapes
    obj = K().getFrameByName(objName)->ID;
    if(R.getRobot()=="pr2"){
      if(rl==LR_right){
        eff = K().getFrameByName("pr2R")->ID;
        grasp1 = K().getFrameByName("r_gripper_joint")->ID;
        grasp2 = K().getFrameByName("r_gripper_l_finger_joint")->ID;
      }else{
        eff = K().getFrameByName("pr2L")->ID;
        grasp1 = K().getFrameByName("l_gripper_joint")->ID;
        grasp2 = K().getFrameByName("l_gripper_l_finger_joint")->ID;
      }
      //cam = K().getShapeByName("endeffKinect")->index;
      //workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention, gripper positioning, alignment, open gripper
    auto look = R.lookAt(objName);
    //auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    //R.wait(1.);
    auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff, NoVector, obj), {}, {0.,0.,above+.1});
#if 1
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_x, obj, Vector_y) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_y, obj, Vector_x) );
#else
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_x, obj, Vector_x) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_y, obj, Vector_y) );
#endif
    double gripSize = width + .10;
    double gripSize2 = ::asin(gripSize/(2.*.10));
    CHECK_EQ(gripSize2, gripSize2, "the object is too think to be grasped??");
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {gripSize2});
    R.wait({-pos, -gripperR, -gripper2R, -up});

    //lowering
    pos->set()->setTarget( ARR(0,0,above) );
    pos->resetStatus();
    R.wait({-pos});

    pos->stop();//don't control obj position when closing gripper
    look->stop();
    //ws->stop();

    //close gripper
    gripSize = width-.015;//+.015;
    gripperR->set()->setTarget( {gripSize} );
    gripper2R->set()->setTarget( {::asin(gripSize/(2.*.10))} );
    gripperR->resetStatus();
    gripper2R->resetStatus();
    R.wait({-gripperR, -gripper2R});
    R.wait(.5);
  }

  {
    //switch
    const char* effName = R.getK()->frames(eff)->name;
    R.kinematicSwitch(objName, effName, false);
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift->set()->setTarget(lift->task->y);
    lift->set()->PD().setGains(0, 10.);
    lift->set()->PD().v_target = ARR(0,0,.2);
    auto look = R.lookAt(objName);
    R.wait(1.);
  }
  return AS_done;
}

int Script_place(Roopi& R, const char* objName, const char* ontoName, const mlr::Quaternion& rot){

  //query some info from the kinematics first
  double width, above;
  //uint obj, onto, eff, grasp1, grasp2, workspace;
  uint obj, onto, eff, grasp1, grasp2;
  {
    auto K = R.getK();

    //get obj size
    arr objSize = K().getFrameByName(objName)->shape->size;
    arr ontoSize = K().getFrameByName(ontoName)->shape->size;
    //width = objSize(1);
    width = objSize(0);
    above = .5*objSize(2)+.5*ontoSize(2);

    //relevant shapes
    mlr::Frame *ob = K().getFrameByName(objName);
    obj = ob->ID;
    onto = K().getFrameByName(ontoName)->ID;
    if(R.getRobot()=="pr2"){
      mlr::Frame *sh = K().getFrameByName("pr2R");
      if(sh->ID == ob->link->from->ID){ //this is the right hand..
        eff = sh->ID;
        grasp1 = K().getFrameByName("r_gripper_joint")->ID;
        grasp2 = K().getFrameByName("r_gripper_l_finger_joint")->ID;
      }else{
        sh = K().getFrameByName("pr2L");
        if(sh->ID == ob->link->from->ID){ //this is the left hand..
          eff = sh->ID;
          grasp1 = K().getFrameByName("l_gripper_joint")->ID;
          grasp2 = K().getFrameByName("l_gripper_l_finger_joint")->ID;
        }else{
          HALT("which hand is this? Something's wrong");
        }
      }
      //workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
    auto look = R.lookAt(objName);
    //auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    //R.wait(1.);
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {0.,0.,above+.1});
#if 1
    auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_x) );
#else
    auto quat = R.newCtrlTask(new TaskMap_Default(quatTMT, obj) );
    quat->set()->setTarget(rot.getArr4d());
#endif
    //R.wait({-ws, -up, -pos});
    R.wait({/*-up,*/ -pos});

    //lowering
    pos->set()->setTarget( ARR(0,0,above) );
    pos->resetStatus();
    R.wait({-pos});

    //switch
    pos->stop();//don't control obj position during kinematic switch
    look->stop();
    //ws->stop();
    R.kinematicSwitch(objName, ontoName, true);

    //open gripper
    double gripSize = width + .05;
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    auto look2 = R.lookAt(objName);
    R.wait({-gripperR, -gripper2R});
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift->set()->setTarget(lift->task->y);
    lift->set()->PD().setGains(0, 10.);
    lift->set()->PD().v_target = ARR(0,0,.2);
    R.wait(1.);
  }
  return AS_done;
}

int Script_placeDistDir(Roopi& R, const char* objName, const char* ontoName, double deltaX, double deltaY, int deltaTheta){

  //query some info from the kinematics first
  double width, above;
  //uint obj, onto, eff, grasp1, grasp2, workspace;
  uint obj, onto, eff, grasp1, grasp2;
  {
    auto K = R.getK();

    //get obj size
    arr objSize = K().getFrameByName(objName)->shape->size;
    arr ontoSize = K().getFrameByName(ontoName)->shape->size;
    //width = objSize(1);
    width = objSize(0);
    above = .5*objSize(2)+.5*ontoSize(2);

    //relevant shapes
    mlr::Frame *ob = K().getFrameByName(objName);
    obj = ob->ID;
    onto = K().getFrameByName(ontoName)->ID;
    if(R.getRobot()=="pr2"){
      mlr::Frame *sh = K().getFrameByName("pr2R");
      if(sh->ID == ob->link->from->ID){ //this is the right hand..
        eff = sh->ID;
        grasp1 = K().getFrameByName("r_gripper_joint")->ID;
        grasp2 = K().getFrameByName("r_gripper_l_finger_joint")->ID;
      }else{
        sh = K().getFrameByName("pr2L");
        if(sh->ID == ob->link->from->ID){ //this is the left hand..
          eff = sh->ID;
          grasp1 = K().getFrameByName("l_gripper_joint")->ID;
          grasp2 = K().getFrameByName("l_gripper_l_finger_joint")->ID;
        }else{
          HALT("which hand is this? Something's wrong");
        }
      }
      //workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
    auto look = R.lookAt(objName);
    //auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    //R.wait(1.);
    //auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {deltaX,deltaY,above+.1});
    auto al1 = R.newCtrlTask();
    auto al2 = R.newCtrlTask();
    auto al3 = R.newCtrlTask();
    if (deltaTheta==0){
      al1->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_z) );
      al3->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    } else if (deltaTheta==1){
      al1->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_z) );
      al3->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_x) );
    } else if (deltaTheta==2){
      al1->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_z, onto, Vector_z) );
      al3->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    } else if (deltaTheta==3){
      al1->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_z, onto, Vector_z) );
      al3->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_x) );
    }
    al1->set()->PD().setGainsAsNatural(1., .9);
    al1->start();
    al2->set()->PD().setGainsAsNatural(1., .9);
    al2->start();
    al3->set()->PD().setGainsAsNatural(1., .9);
    al3->start();
    //R.wait({&ws, &up, &pos});
    //R.wait({&up, &pos});
    R.wait({-pos});

    //lowering
    pos->set()->setTarget( ARR(deltaX,deltaY,above) );
    pos->resetStatus();
    R.wait({-pos});

    //switch
    pos->stop();//don't control obj position during kinematic switch
    look->stop();
    //ws->stop();
    R.kinematicSwitch(objName, ontoName, true);

    //open gripper
    double gripSize = width + .05;
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    auto look2 = R.lookAt(objName);
    R.wait({-gripperR, -gripper2R});
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift->set()->setTarget(lift->task->y);
    lift->set()->PD().setGains(0, 10.);
    lift->set()->PD().v_target = ARR(0,0,.2);
    R.wait(1.);
  }
  return AS_done;
}

int Script_armsNeutral(Roopi& R){
  //query some info from the kinematics first
  uint shoulderPanJointR, shoulderLiftJointR, upperArmRollJointR, elbowFlexJointR, forearmRollJointR;
  uint shoulderPanJointL, shoulderLiftJointL, upperArmRollJointL, elbowFlexJointL, forearmRollJointL;
  {
    auto K = R.getK();
    if(R.getRobot()=="pr2"){
        shoulderPanJointR= K().getFrameByName("r_shoulder_pan_joint")->ID;
        shoulderLiftJointR= K().getFrameByName("r_shoulder_lift_joint")->ID;
        upperArmRollJointR= K().getFrameByName("r_upper_arm_roll_joint")->ID;
        elbowFlexJointR= K().getFrameByName("r_elbow_flex_joint")->ID;
        forearmRollJointR= K().getFrameByName("r_forearm_roll_joint")->ID;
        shoulderPanJointL= K().getFrameByName("l_shoulder_pan_joint")->ID;
        shoulderLiftJointL= K().getFrameByName("l_shoulder_lift_joint")->ID;
        upperArmRollJointL= K().getFrameByName("l_upper_arm_roll_joint")->ID;
        elbowFlexJointL= K().getFrameByName("l_elbow_flex_joint")->ID;
        forearmRollJointL= K().getFrameByName("l_forearm_roll_joint")->ID;
    }else{
      NIY;
    }
  }

  {
      auto shoulderPanR =  R.newCtrlTask(new TaskMap_qItself({shoulderPanJointR}, false), {}, {-1.5});  //-1
      auto shoulderLiftR =  R.newCtrlTask(new TaskMap_qItself({shoulderLiftJointR}, false), {}, {-0.5});  //0.5
      auto upperArmRollR =  R.newCtrlTask(new TaskMap_qItself({upperArmRollJointR}, false), {}, {-1});   //-1
      auto elbowFlexR =  R.newCtrlTask(new TaskMap_qItself({elbowFlexJointR}, false), {}, {-2});   //-2
      auto forearmRollR =  R.newCtrlTask(new TaskMap_qItself({forearmRollJointR}, false), {}, {-1.5});   //-1.5
      auto shoulderPanL =  R.newCtrlTask(new TaskMap_qItself({shoulderPanJointL}, false), {}, {1.5});  //1
      auto shoulderLiftL =  R.newCtrlTask(new TaskMap_qItself({shoulderLiftJointL}, false), {}, {-0.5});  //0.5
      auto upperArmRollL =  R.newCtrlTask(new TaskMap_qItself({upperArmRollJointL}, false), {}, {1});   //1
      auto elbowFlexL =  R.newCtrlTask(new TaskMap_qItself({elbowFlexJointL}, false), {}, {-2});   //-2
      auto forearmRollL =  R.newCtrlTask(new TaskMap_qItself({forearmRollJointL}, false), {}, {1.5});   //1.5
      R.wait({-shoulderPanR,-shoulderLiftR,-upperArmRollR,-elbowFlexR,-forearmRollR});
      R.wait({-shoulderPanL,-shoulderLiftL,-upperArmRollL,-elbowFlexL,-forearmRollL});
  }
  return AS_done;
}


int Script_workspaceReady(Roopi& R, const char* objName){
  uint obj, workspace;
  {
    auto K = R.getK();

    //relevant shapes
    mlr::Frame *ob = K().getFrameByName(objName);
    obj = ob->ID;
    if(R.getRobot()=="pr2"){
//      mlr::Shape *sh = K().getShapeByName("pr2R");
      workspace = K().getFrameByName("endeffWorkspace")->ID;
    }else{
      NIY;
    }
  }
  {
    //attention & workspace positioning
    auto look = R.lookAt(objName);
    auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    R.wait({-ws});
  }
  return AS_done;
}




int Script_komoGraspBox(Roopi& R, const char* objName, LeftOrRight rl){

  const char *endeff, *gripper, *gripper2, *group1, *group2;
  if(rl==LR_right){
    endeff="pr2R";
    gripper="r_gripper_joint"; gripper2="r_gripper_l_finger_joint";
    group1="armR"; group2="gripR";
  }else{
    endeff="pr2L";
    gripper="l_gripper_joint"; gripper2="l_gripper_l_finger_joint";
    group1="armL"; group2="gripL";
  }

  arr obj1size = R.getK()->getFrameByName(objName)->shape->size;
  double gripSize = obj1size(0);
  double above = .5*obj1size(2) - .05;

  {
    auto path = R.newPathOpt();
    path->komo->useJointGroups({group1, group2});
    path->komo->setPathOpt(1, 20, 5.);
    path->komo->setFine_grasp(1., endeff, objName, above, gripSize, gripper, gripper2);
    path->start();

    R.wait({-path});

    auto follow = Act_FollowPath(&R, "PathFollower", path->komo->x, new TaskMap_qItself(QIP_byJointGroups, {group1, group2}, R.getK()), 5.);
    follow.start();

    R.wait({&follow});
  }

  {
    double gripSize = getGripSize(R, "obj1");
    auto closeGrip = R.setGripper(LR_left, gripSize-.05);
    R.wait(+closeGrip);
  }

  R.kinematicSwitch("obj1", "pr2L", false);

  {
    auto lift = R.moveVel("pr2L", {0,0,.2});
    R.wait(1.);
  }

  return AS_done;
}


double getGripSize(Roopi& R, const char* objName, CubeSide cs){
  arr obj1size = R.getK()->getFrameByName(objName)->shape->size;
  return obj1size(cs) + 2.*obj1size(3);
}
