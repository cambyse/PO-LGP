#include "script_PickAndPlace.h"
#include "roopi.h"
#include <Control/taskControl.h>
#include <Motion/komo.h>

int Script_setGripper(Roopi& R, LeftOrRight lr, double gripSize){
  //query some info from the kinematics first
  uint grasp1, grasp2;
  {
    auto K = R.getK();
    if(R.getRobot()=="pr2"){
      if(lr==LR_right){
        grasp1 = K().getJointByName("r_gripper_joint")->to->index;
        grasp2 = K().getJointByName("r_gripper_l_finger_joint")->to->index;
      }else{
        grasp1 = K().getJointByName("l_gripper_joint")->to->index;
        grasp2 = K().getJointByName("l_gripper_l_finger_joint")->to->index;
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
  uint obj, eff, grasp1, grasp2;
  {
    auto K = R.getK();

    //get obj size
    arr objSize = K().getShapeByName(objName)->size;
    width = objSize(0);
    above = .5*objSize(2);

    //relevant shapes
    obj = K().getShapeByName(objName)->index;
    if(R.getRobot()=="pr2"){
      if(rl==LR_right){
        eff = K().getShapeByName("pr2R")->index;
        grasp1 = K().getJointByName("r_gripper_joint")->to->index;
        grasp2 = K().getJointByName("r_gripper_l_finger_joint")->to->index;
      }else{
        eff = K().getShapeByName("pr2L")->index;
        grasp1 = K().getJointByName("l_gripper_joint")->to->index;
        grasp2 = K().getJointByName("l_gripper_l_finger_joint")->to->index;
      }
    }else{
      NIY;
    }
  }

  {
    //attention, gripper positioning, alignment, open gripper
    auto look = R.lookAt(objName);
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
    pos->set()->PD().setTarget( ARR(0,0,above-.01) );
    pos->resetStatus();
    R.wait({-pos});

    pos->stop();//don't control obj position when closing gripper
    look->stop();

    //close gripper
    gripSize = width-.015;//+.015;
    gripperR->set()->PD().setTarget( {gripSize} );
    gripper2R->set()->PD().setTarget( {::asin(gripSize/(2.*.10))} );
    gripperR->resetStatus();
    gripper2R->resetStatus();
    R.wait({-gripperR, -gripper2R});
    R.wait(.5);
  }

  {
    //switch
    const char* effName = R.getK()->shapes(eff)->name;
    R.kinematicSwitch(objName, effName, false);
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift->set()->PD().setTarget(lift->task->y);
    lift->set()->PD().setGains(0, 10.);
    lift->set()->PD().v_target = ARR(0,0,.2);
    auto look = R.lookAt(objName);
    R.wait(1.);
  }
  return AS_done;
}

int Script_pointBox(Roopi& R, const char* objName, LeftOrRight rl){
  //query some info from the kinematics first
  double above;
  uint obj, eff;
  {
    auto K = R.getK();
    //get obj size
    arr objSize = K().getShapeByName(objName)->size;
    above = .5*objSize(2);
    //relevant shapes
    obj = K().getShapeByName(objName)->index;
    if(R.getRobot()=="pr2"){
      if(rl==LR_right){
        eff = K().getShapeByName("pr2R")->index;
      }else{
        eff = K().getShapeByName("pr2L")->index;
      }
    }else{
      NIY;
    }
  }
  {
    //attention, positioning
    auto look = R.lookAt(objName);
    auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff, NoVector, obj), {}, {0.,0.,above+.1});
    R.wait({-pos, -up});
  }
  return AS_done;
}

int Script_place(Roopi& R, const char* objName, const char* ontoName, const mlr::Quaternion& rot){

  //query some info from the kinematics first
  double width, above;
  uint obj, onto, eff, grasp1, grasp2;
  {
    auto K = R.getK();

    //get obj size
    arr objSize = K().getShapeByName(objName)->size;
    arr ontoSize = K().getShapeByName(ontoName)->size;
    width = objSize(0);
    above = .5*objSize(2)+.5*ontoSize(2);

    //relevant shapes
    mlr::Shape *ob = K().getShapeByName(objName);
    obj = ob->index;
    onto = K().getShapeByName(ontoName)->index;
    if(R.getRobot()=="pr2"){
      mlr::Shape *sh = K().getShapeByName("pr2R");
      if(sh->body->index == ob->body->inLinks.scalar()->from->index){ //this is the right hand..
        eff = sh->index;
        grasp1 = K().getJointByName("r_gripper_joint")->to->index;
        grasp2 = K().getJointByName("r_gripper_l_finger_joint")->to->index;
      }else{
        sh = K().getShapeByName("pr2L");
        if(sh->body->index == ob->body->inLinks.scalar()->from->index){ //this is the left hand..
          eff = sh->index;
          grasp1 = K().getJointByName("l_gripper_joint")->to->index;
          grasp2 = K().getJointByName("l_gripper_l_finger_joint")->to->index;
        }else{
          HALT("which hand is this? Something's wrong");
        }
      }
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
    auto look = R.lookAt(objName);
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {0.,0.,above+.1});
#if 1
    auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_x) );
#else
    auto quat = R.newCtrlTask(new TaskMap_Default(quatTMT, obj) );
    quat->set()->PD().setTarget(rot.getArr4d());
#endif
    R.wait({-pos});

    //lowering
    pos->set()->PD().setTarget( ARR(0,0,above) );
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
    lift->set()->PD().setTarget(lift->task->y);
    lift->set()->PD().setGains(0, 10.);
    lift->set()->PD().v_target = ARR(0,0,.2);
    R.wait(1.);
  }
  return AS_done;
}

int Script_placeDistDir(Roopi& R, const char* objName, const char* ontoName, double deltaX, double deltaY, double deltaZ, int deltaTheta){

  //query some info from the kinematics first
  double width, above;
  uint obj, onto, eff, grasp1, grasp2;
  {
    auto K = R.getK();

    //get obj size
    arr objSize = K().getShapeByName(objName)->size;
    arr ontoSize = K().getShapeByName(ontoName)->size;
    width = objSize(0);
    above = .5*objSize(2)+.5*ontoSize(2);

    //relevant shapes
    mlr::Shape *ob = K().getShapeByName(objName);
    obj = ob->index;
    onto = K().getShapeByName(ontoName)->index;
    if(R.getRobot()=="pr2"){
      mlr::Shape *sh = K().getShapeByName("pr2R");
      if(sh->body->index == ob->body->inLinks.scalar()->from->index){ //this is the right hand..
        eff = sh->index;
        grasp1 = K().getJointByName("r_gripper_joint")->to->index;
        grasp2 = K().getJointByName("r_gripper_l_finger_joint")->to->index;
      }else{
        sh = K().getShapeByName("pr2L");
        if(sh->body->index == ob->body->inLinks.scalar()->from->index){ //this is the left hand..
          eff = sh->index;
          grasp1 = K().getJointByName("l_gripper_joint")->to->index;
          grasp2 = K().getJointByName("l_gripper_l_finger_joint")->to->index;
        }else{
          HALT("which hand is this? Something's wrong");
        }
      }
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
    auto look = R.lookAt(objName);
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {deltaX,deltaY,above+.1+deltaZ});
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
    R.wait({-pos});

    //lowering
    pos->set()->PD().setTarget( ARR(deltaX,deltaY,above+deltaZ) );
    pos->resetStatus();
    R.wait({-pos});

    //switch
    pos->stop();//don't control obj position during kinematic switch
    look->stop();
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
    lift->set()->PD().setTarget(lift->task->y);
    lift->set()->PD().setGains(0, 10.);
    lift->set()->PD().v_target = ARR(0,0,.2);
    R.wait(1.);
  }
  return AS_done;
}

int Script_pointPosition(Roopi& R, const char* objName, const char* ontoName, LeftOrRight rl, double deltaX, double deltaY, double deltaZ){
  //assume objName in hand, place at a position relative to ontoName and keep in gripper
  //query some info from the kinematics first
  double above;
  uint obj, onto;
  {
    auto K = R.getK();
    //get obj size
    above = 0.06;
    //relevant shapes
    obj = K().getShapeByName(objName)->index;
    onto = K().getShapeByName(ontoName)->index;
  }
  {
    //attention, positioning
    auto look = R.lookAt(ontoName);
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {deltaX,deltaY,above+deltaZ});
    auto al1 = R.newCtrlTask();
    auto al2 = R.newCtrlTask();
    auto al3 = R.newCtrlTask();
    al1->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
    al2->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_z) );
    al3->setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    al1->set()->PD().setGainsAsNatural(1., .9);
    al1->start();
    al2->set()->PD().setGainsAsNatural(1., .9);
    al2->start();
    al3->set()->PD().setGainsAsNatural(1., .9);
    al3->start();
    R.wait(+pos);
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
        shoulderPanJointR= K().getJointByName("r_shoulder_pan_joint")->to->index;
        shoulderLiftJointR= K().getJointByName("r_shoulder_lift_joint")->to->index;
        upperArmRollJointR= K().getJointByName("r_upper_arm_roll_joint")->to->index;
        elbowFlexJointR= K().getJointByName("r_elbow_flex_joint")->to->index;
        forearmRollJointR= K().getJointByName("r_forearm_roll_joint")->to->index;
        shoulderPanJointL= K().getJointByName("l_shoulder_pan_joint")->to->index;
        shoulderLiftJointL= K().getJointByName("l_shoulder_lift_joint")->to->index;
        upperArmRollJointL= K().getJointByName("l_upper_arm_roll_joint")->to->index;
        elbowFlexJointL= K().getJointByName("l_elbow_flex_joint")->to->index;
        forearmRollJointL= K().getJointByName("l_forearm_roll_joint")->to->index;
//        torsoLiftJoint= K().getJointByName("torso_lift_joint")->to->index;
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
//      auto torsoLift = R.newCtrlTask(new TaskMap_qItself({torsoLiftJoint}, false), {}, {0.3});
      R.wait({-shoulderPanR,-shoulderLiftR,-upperArmRollR,-elbowFlexR,-forearmRollR});
      R.wait({-shoulderPanL,-shoulderLiftL,-upperArmRollL,-elbowFlexL,-forearmRollL});
//      R.wait({-torsoLift});
  }
  return AS_done;
}

int Script_setTorso(Roopi& R, int up_down){
  uint torsoLiftJoint;
  {
    auto K = R.getK();
    if(R.getRobot()=="pr2"){
      torsoLiftJoint= K().getJointByName("torso_lift_joint")->to->index;
    }else{
      NIY;
    }
  }
  {
    auto torsoLift = R.newCtrlTask(new TaskMap_qItself({torsoLiftJoint}, false), {}, {0.3*up_down});
    R.wait({-torsoLift});
  }
  return AS_done;
}

int Script_workspaceReady(Roopi& R, const char* objName){
  uint obj, workspace;
  {
    auto K = R.getK();

    //relevant shapes
    mlr::Shape *ob = K().getShapeByName(objName);
    obj = ob->index;
    if(R.getRobot()=="pr2"){
      workspace = K().getShapeByName("endeffWorkspace")->index;
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

  arr obj1size = R.getK()->getShapeByName(objName)->size;
  double gripSize = obj1size(1) + 2.*obj1size(3);
  double above = obj1size(2)*.5 + obj1size(3) - .02;

  auto path = R.newPathOpt();
  path->komo->useOnlyJointGroup({group1, group2});
  path->komo->setPathOpt(1, 20, 5.);
  path->komo->setFine_grasp(1., endeff, objName, above, gripSize, gripper, gripper2);
  path->start();

  R.wait({-path});

  auto follow = Act_FollowPath(&R, "PathFollower", path->komo->x, new TaskMap_qItself(QIP_byJointGroups, {group1, group2}, R.getK()), 5.);
  follow.start();

  R.wait({&follow});

  return AS_done;
}


double getGripSize(Roopi& R, const char* objName, CubeSide cs){
  arr obj1size = R.getK()->getShapeByName(objName)->size;
  return obj1size(cs) + 2.*obj1size(3);
}
