#include <Roopi/roopi.h>
#include <Control/taskControl.h>


//===============================================================================

void homing() {
  Roopi R(true);
  {
    auto h = R.home();
    R.wait(+h);
  }
}

//===============================================================================

void testCompliant() {
  {
    Roopi R(true);
    R.getTaskController().lockJointGroupControl("base");

    R.collisions(true);

    {
      auto h = R.home();
      R.wait(+h);
      R.wait();
    }

    {
      auto posL = R.newCtrlTask();
      posL->setMap(new TaskMap_Default(posTMT, R.getK(), "endeffL"));
      posL->task->complianceDirection = {1., 0., .0};
      posL->task->PD().setTarget( posL->y0 );
      posL->task->PD().setGainsAsNatural(1., .9);
      posL->start();

//      R.wait(+posL);
      R.wait();
    }

    {
      auto h = R.home();
      R.wait(+h);
    }
  }
  cout <<"LEFT OVER REGISTRY:\n" <<registry() <<endl;
}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  homing();
  testCompliant();

  return 0;
}

