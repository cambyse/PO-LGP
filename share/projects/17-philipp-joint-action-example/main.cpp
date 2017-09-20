#include "../17-philipp-HRI-task/HRI_state.h"
#include <Roopi/roopi.h>
#include <Control/taskControl.h>
#include <Motion/komo.h>
#include <RosCom/subscribeRosKinect.h>
#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>
#include <Core/util.h>
#include <iostream>


int placeDistDir1(Roopi& R, const char* objName, const char* ontoName, double deltaX, double deltaY, double deltaZ, int deltaTheta){

  //query some info from the kinematics first
  double above;
  //uint obj, onto, eff, grasp1, grasp2, workspace;
  uint obj, onto;
  {
    auto K = R.getK();
    mlr::Shape *ob = K().getShapeByName(objName);
    obj = ob->index;
    onto = K().getShapeByName(ontoName)->index;
    //get obj size
    arr objSize = K().getShapeByName(objName)->size;
    arr ontoSize = K().getShapeByName(ontoName)->size;
    //width = objSize(1);
    above = .5*objSize(2)+.5*ontoSize(2);
  }
  {
    //attention & gripper positioning
    auto look = R.lookAt(objName);
    //auto ws =   R.newCtrlTask(new TaskMap_Default(posDiffTMT, workspace, NoVector, obj), {}, {}, {2e-1});
    //R.wait(1.);
    //auto up =   R.newCtrlTask(new TaskMap_Default(vecTMT, eff, Vector_z), {}, {0.,0.,1.});
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
    //R.wait({&ws, &up, &pos});
    //R.wait({&up, &pos});
    R.wait({-pos});
    
    //lowering
    pos->set()->PD().setTarget( ARR(deltaX,deltaY,above+deltaZ) );
    pos->resetStatus();
    R.wait({-pos});
    
    pos->stop();//don't control obj position during kinematic switch
    look->stop();
    return AS_done;
  }
}

int placeDistDir2(Roopi& R, const char* objName, const char* ontoName) {
 uint eff, grasp1, grasp2;
  {
    auto K = R.getK();
    mlr::Shape *ob = K().getShapeByName(objName);
    //relevant shapes
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
  R.kinematicSwitch(objName, ontoName, true);  
  //open gripper 
 {
    auto gripperR =  R.newCtrlTask(new TaskMap_qItself({grasp1}, false), {}, {10.});
    auto gripper2R = R.newCtrlTask(new TaskMap_qItself({grasp2}, false), {}, {10.});
    auto look2 = R.lookAt(objName);
    R.wait({-gripperR, -gripper2R});
  }
  //lift hand
  auto lift = R.newCtrlTask(new TaskMap_Default(posDiffTMT, eff));
  lift->set()->PD().setTarget(lift->task->y);
  lift->set()->PD().setGains(0, 10.);
  lift->set()->PD().v_target = ARR(0,0,.2);
  R.wait(1.);
  return AS_done;
}




int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  //  init
  Roopi R(true,false);
  R.getTaskController().lockJointGroupControl("base");
  R.newMarker("center", ARR(.75,.0,.6));
  int use_ros = mlr::getParameter<int>("useRos", 0);
  auto pcl = R.PclPipeline(false);
  OnTableState state;
  SubscribeRosKinect subKin; //subscription into depth and rgb images
  ImageViewer v2("kinect_rgb");

  // look at center and arms Neutral
  {
    auto L = R.lookAt("center", .1);
    auto an = R.armsNeutral();
    R.wait({-L, -an});
    R.wait(.1);
  }
  R.wait(15.);
  // init state
  {
    Access<PerceptL> outputs("percepts_input");
    int rev=outputs.getRevision();  
    outputs.waitForRevisionGreaterThan(rev+5);
    state.initFromPercepts(outputs.get());
    state.updateModelworldFromState();
  }

  // open viewer to see the modelworld
  OrsViewer viewer1("modelWorld");

  // query objects and place blue large on the red small object with offset
  HRIObject* bl = state.getObj(HRIObject::blue, HRIObject::large);
  HRIObject* rs = state.getObj(HRIObject::red, HRIObject::small);
  auto graspObj = R.graspBox(bl->id.c_str(), LR_left);
  R.wait(+graspObj);
  auto placeObj = placeDistDir1(R, bl->id.c_str(),rs->id.c_str(),0.,.05,0.,1);
  R.wait(+placeObj);

  // use a new state to check whether red medium object is near to red small
  while (true) {
    OnTableState state2;
    Access<PerceptL> outputs("percepts_input");
    int rev=outputs.getRevision();  
    outputs.waitForRevisionGreaterThan(rev+2);
    state2.initFromPercepts(outputs.get());
    HRIObject* rm = state2.getObj(HRIObject::red, HRIObject::medium);
    cout << state2 << endl;
    if (rm!=NULL) {
      cout << sqrt((rm->pos.x-rs->pos.x)*(rm->pos.x-rs->pos.x)+(rm->pos.y-rs->pos.y)*(rm->pos.y-rs->pos.y)) << endl;
      if ((sqrt((rm->pos.x-rs->pos.x)*(rm->pos.x-rs->pos.x)+(rm->pos.y-rs->pos.y)*(rm->pos.y-rs->pos.y)) < 0.1))
	break;
    }
  }
  auto placeObj2= placeDistDir2(R, bl->id.c_str(),rs->id.c_str() );
  R.wait(+placeObj2);

  // look at center and arms Neutral
  {
    auto L = R.lookAt("center", .1);
    auto an = R.armsNeutral();
    R.wait({-L, -an});
    R.wait(.1);
  }
}
