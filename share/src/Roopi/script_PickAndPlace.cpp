#include "script_PickAndPlace.h"
#include "roopi.h"
#include <Control/taskControl.h>

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
    R.wait({&gripperR, &gripper2R});
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
    arr objSize = K().getShapeByName(objName)->size;
    //width = objSize(1);
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
    //mlr::wait(1.);
    auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff, NoVector, obj), {}, {0.,0.,above+.1});
#if 1
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_x, obj, Vector_y) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_y, obj, Vector_x) );
#else
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_x, obj, Vector_x) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, eff, Vector_y, obj, Vector_y) );
#endif
    double gripSize = width + .05;
    double gripSize2 = ::asin(gripSize/(2.*.10));
    CHECK_EQ(gripSize2, gripSize2, "the object is too think to be grasped??");
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {gripSize2});
    R.wait({&pos, &gripperR, &gripper2R, &up});

    //lowering
    pos.set()->PD().setTarget( ARR(0,0,above-.01) );
    pos.resetStatus();
    R.wait({&pos});

    pos.stop();//don't control obj position when closing gripper
    look.stop();
    //ws.stop();

    //close gripper
    gripSize = width-.01;//+.015;
    gripperR.set()->PD().setTarget( {gripSize} );
    gripper2R.set()->PD().setTarget( {::asin(gripSize/(2.*.10))} );
    gripperR.resetStatus();
    gripper2R.resetStatus();
    R.wait({&gripperR, &gripper2R});
    mlr::wait(.5);
  }

  {
    //switch
    const char* effName = R.getK()->shapes(eff)->name;
    R.kinematicSwitch(objName, effName);
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift.set()->PD().setTarget(lift.task->y);
    lift.set()->PD().setGains(0, 10.);
    lift.set()->PD().v_target = ARR(0,0,.2);
    auto look = R.lookAt(objName);
    mlr::wait(1.);
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
    arr objSize = K().getShapeByName(objName)->size;
    arr ontoSize = K().getShapeByName(ontoName)->size;
    //width = objSize(1);
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
      //workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
    auto look = R.lookAt(objName);
    //auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    //mlr::wait(1.);
//    auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {0.,0.,above+.1});
#if 0
    auto al1 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    auto al2 = R.newCtrlTask(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_x) );
#else
    auto quat = R.newCtrlTask(new TaskMap_Default(quatTMT, obj) );
    quat.set()->PD().setTarget(rot.getArr4d());
#endif
    //R.wait({&ws, &up, &pos});
    R.wait({/*&up,*/ &pos});

    //lowering
    pos.set()->PD().setTarget( ARR(0,0,above) );
    pos.resetStatus();
    R.wait({&pos});

    //switch
    pos.stop();//don't control obj position during kinematic switch
    look.stop();
    //ws.stop();
    R.kinematicSwitch(objName, ontoName);

    //open gripper
    double gripSize = width + .05;
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    auto look2 = R.lookAt(objName);
    R.wait({&gripperR, &gripper2R});
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift.set()->PD().setTarget(lift.task->y);
    lift.set()->PD().setGains(0, 10.);
    lift.set()->PD().v_target = ARR(0,0,.2);
    mlr::wait(1.);
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
    arr objSize = K().getShapeByName(objName)->size;
    arr ontoSize = K().getShapeByName(ontoName)->size;
    //width = objSize(1);
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
      //workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
    auto look = R.lookAt(objName);
    //auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    //mlr::wait(1.);
    //auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {deltaX,deltaY,above+.1});
    auto al1 = R.newCtrlTask();
    auto al2 = R.newCtrlTask();
    auto al3 = R.newCtrlTask();
    if (deltaTheta==0){
      al1.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_z) );
      al3.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    } else if (deltaTheta==1){
      al1.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_y, onto, Vector_z) );
      al3.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_x) );
    } else if (deltaTheta==2){
      al1.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_z, onto, Vector_z) );
      al3.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_y) );
    } else if (deltaTheta==3){
      al1.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_z) );
      al2.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_z, onto, Vector_z) );
      al3.setMap(new TaskMap_Default(vecAlignTMT, obj, Vector_x, onto, Vector_x) );
    }
    al1.set()->PD().setGainsAsNatural(1., .9);
    al1.start();
    al2.set()->PD().setGainsAsNatural(1., .9);
    al2.start();
    al3.set()->PD().setGainsAsNatural(1., .9);
    al3.start();
    //R.wait({&ws, &up, &pos});
    //R.wait({&up, &pos});
    R.wait({&pos});

    //lowering
    pos.set()->PD().setTarget( ARR(deltaX,deltaY,above) );
    pos.resetStatus();
    R.wait({&pos});

    //switch
    pos.stop();//don't control obj position during kinematic switch
    look.stop();
    //ws.stop();
    R.kinematicSwitch(objName, ontoName);

    //open gripper
    double gripSize = width + .05;
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    auto look2 = R.lookAt(objName);
    R.wait({&gripperR, &gripper2R});
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift.set()->PD().setTarget(lift.task->y);
    lift.set()->PD().setGains(0, 10.);
    lift.set()->PD().v_target = ARR(0,0,.2);
    mlr::wait(1.);
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
    }else{
      NIY;
    }
  }

  {
      auto shoulderPanR =  R.newCtrlTask(new TaskMap_qItself({shoulderPanJointR}, false), {}, {-1});  //-1
      auto shoulderLiftR =  R.newCtrlTask(new TaskMap_qItself({shoulderLiftJointR}, false), {}, {-0.5});  //0.5
      auto upperArmRollR =  R.newCtrlTask(new TaskMap_qItself({upperArmRollJointR}, false), {}, {-1});   //-1
      auto elbowFlexR =  R.newCtrlTask(new TaskMap_qItself({elbowFlexJointR}, false), {}, {-2});   //-2
      auto forearmRollR =  R.newCtrlTask(new TaskMap_qItself({forearmRollJointR}, false), {}, {-1.5});   //-1.5
      auto shoulderPanL =  R.newCtrlTask(new TaskMap_qItself({shoulderPanJointL}, false), {}, {1});  //1
      auto shoulderLiftL =  R.newCtrlTask(new TaskMap_qItself({shoulderLiftJointL}, false), {}, {-0.5});  //0.5
      auto upperArmRollL =  R.newCtrlTask(new TaskMap_qItself({upperArmRollJointL}, false), {}, {1});   //1
      auto elbowFlexL =  R.newCtrlTask(new TaskMap_qItself({elbowFlexJointL}, false), {}, {-2});   //-2
      auto forearmRollL =  R.newCtrlTask(new TaskMap_qItself({forearmRollJointL}, false), {}, {1.5});   //1.5
      R.wait({&shoulderPanR,&shoulderLiftR,&upperArmRollR,&elbowFlexR,&forearmRollR});
      R.wait({&shoulderPanL,&shoulderLiftL,&upperArmRollL,&elbowFlexL,&forearmRollL});
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
//      mlr::Shape *sh = K().getShapeByName("pr2R");
      workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }
  {
    //attention & workspace positioning
    auto look = R.lookAt(objName);
    auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    R.wait({&ws});
  }
  return AS_done;
}
