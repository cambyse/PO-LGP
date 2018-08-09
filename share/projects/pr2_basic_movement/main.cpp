#include <Roopi/roopi.h>
#include <Motion/komo.h>
#include <Control/taskControl.h>
#include <RosCom/subscribeRosKinect.h>
#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>


//===============================================================================

void TEST(BasicMovement){
  {
    Roopi R(true);
    {
      uint torsoLiftJoint=R.getK()().getJointByName("torso_lift_joint")->to->index;
      auto torsoUp =  R.newCtrlTask(new TaskMap_qItself({torsoLiftJoint}, false), {}, {.2});
      R.wait(+torsoUp);
      R.wait();
    }
    {
      auto posL = R.newCtrlTask();
      posL->setMap(new TaskMap_Default(posTMT, R.getK(), "endeffL"));
      posL->task->PD().setTarget( posL->y0 + ARR(0,-.1,-.3) );
      posL->task->PD().setGainsAsNatural(1., .9);
      posL->start();

      R.wait(+posL);
    }

    auto graspObj = R.graspBox("obj1", LR_right);
    R.wait(+graspObj);
    auto placeObj = R.place("obj1","target");
    R.wait(+placeObj);


    {
      auto h = R.home();
      R.wait(+h);
    }
    R.wait();
  }
}





//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testBasicMovement();

  return 0;
}

