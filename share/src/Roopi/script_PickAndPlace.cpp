#include "script_PickAndPlace.h"
#include "roopi-private.h"
#include <Control/taskControl.h>

int Script_graspBox(Roopi& R, const char* objName, bool rightNotLeft){

  //query some info from the kinematics first
  double width, above;
  uint obj, eff, grasp1, grasp2, cam, workspace;
  {
    auto K = R.getKinematics();

    //get obj size
    arr objSize(K().getShapeByName(objName)->size, 4, false);
    width = 2.*objSize(1);
    above = objSize(2);

    //relevant shapes
    obj = K().getShapeByName(objName)->index;
    if(R.s->robot=="pr2"){
      if(rightNotLeft){
        eff = K().getShapeByName("pr2R")->index;
        grasp1 = K().getJointByName("r_gripper_joint")->to->index;
        grasp2 = K().getJointByName("r_gripper_l_finger_joint")->to->index;
      }else{
        eff = K().getShapeByName("pr2L")->index;
        grasp1 = K().getJointByName("l_gripper_joint")->to->index;
        grasp2 = K().getJointByName("l_gripper_l_finger_joint")->to->index;
      }
      cam = K().getShapeByName("endeffKinect")->index;
      workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention, gripper positioning, alignment, open gripper
//    auto look = R.newCtrlTask(new TaskMap_Default(gazeAtTMT, cam, NoVector, obj));
    R.lookAt(objName);
    auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {1e-2});
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
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    R.hold(false);
    R.wait({&pos, &gripperR, &ws, &up});

    //lowering
    pos.set()->PD().setTarget( ARR(0,0,above-.05) );
    R.wait({&pos});

    //close gripper
    gripSize = width+.015;
    gripperR.set()->PD().setTarget( {gripSize} );
    gripper2R.set()->PD().setTarget( {::asin(gripSize/(2.*.10))} );
    R.wait({&pos, &gripperR, &gripper2R});

    //switch
    const char* effName = R.getKinematics()->shapes(eff)->name;
    R.kinematicSwitch(objName, effName);
    R.hold(true);
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift.set()->PD().setTarget(lift.task->y);
    lift.set()->PD().setGains(0, 10.);
    lift.set()->PD().v_target = ARR(0,0,.2);
    R.lookAt(objName);
    R.hold(false);
    mlr::wait(1.);
    R.hold(true);
  }
  return AS_done;
}

int Script_place(Roopi& R, const char* objName, const char* ontoName){

  //query some info from the kinematics first
  double width, above;
  uint obj, onto, eff, grasp1, grasp2, cam, workspace;
  {
    auto K = R.getKinematics();

    //get obj size
    arr objSize(K().getShapeByName(objName)->size, 4, false);
    arr ontoSize(K().getShapeByName(ontoName)->size, 4, false);
    width = 2.*objSize(1);
    above = objSize(2)+ontoSize(2);

    //relevant shapes
    mlr::Shape *ob = K().getShapeByName(objName);
    obj = ob->index;
    onto = K().getShapeByName(ontoName)->index;
    if(R.s->robot=="pr2"){
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
      cam = K().getShapeByName("endeffKinect")->index;
      workspace = K().getShapeByName("endeffWorkspace")->index;
    }else{
      NIY;
    }
  }

  {
    //attention & gripper positioning
//    auto look = R.newCtrlTask(new TaskMap_Default(gazeAtTMT, cam, NoVector, obj));
    R.lookAt(objName);
    auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {1e1});
    auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
    auto pos =  R.newCtrlTask(new TaskMap_Default(posDiffTMT, obj, NoVector, onto), {2.,.9}, {0.,0.,above+.1});
    R.hold(false);
    R.wait({&ws, &up, &pos});

    //lowering
    pos.set()->PD().setTarget( ARR(0,0,above) );
    R.wait({&pos});

    //switch
    R.kinematicSwitch(objName, ontoName);

    //open gripper
    double gripSize = width + .05;
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {gripSize});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {::asin(gripSize/(2.*.10))});
    R.lookAt(objName);
    R.wait({&gripperR, &gripper2R});
  }

  {
    //lift hand
    auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
    lift.set()->PD().setTarget(lift.task->y);
    lift.set()->PD().setGains(0, 10.);
    lift.set()->PD().v_target = ARR(0,0,.2);
    R.hold(false);
    mlr::wait(1.);
    R.hold(true);
  }
  return AS_done;
}
