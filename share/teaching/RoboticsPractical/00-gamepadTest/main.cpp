#include <Core/thread.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsViewer.h>
#include <RosCom/baxter.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("gamepadControl");

  Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");

  TaskControllerModule tcm("baxter");
  GamepadInterface gamepad;
  GamepadControlActivity gpc;

  OrsViewer view;

  if(mlr::getParameter<bool>("useRos")){
    new SendPositionCommandsToBaxter();
    new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
  }

  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  //NodeL subs = registry().getNodesOfType<SubscriberType*>();
  //for(Node *n:subs){ delete n->get<SubscriberType*>(); delete n; }
  cout <<"bye bye" <<endl;
  return 0;
}
