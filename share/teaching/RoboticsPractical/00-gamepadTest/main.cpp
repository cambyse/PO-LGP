#include <Core/thread.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControlThread.h>
#include <Hardware/gamepad/gamepad.h>
#include <Kin/kinViewer.h>
#include <RosCom/baxter.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("gamepadControl");

  Access<sensor_msgs::JointState> jointState(NULL, "jointState");

  TaskControlThread tcm("baxter");
  GamepadInterface gamepad;
  GamepadControlActivity gpc;

  OrsViewer view;

  if(mlr::getParameter<bool>("useRos")){
    new SendPositionCommandsToBaxter();
    new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
  }

  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  threadOpenModules(true);

  moduleShutdown()->waitForStatusGreaterThan(0);

  threadCloseModules();

  //NodeL subs = registry()->getNodesOfType<SubscriberType*>();
  //for(Node *n:subs){ delete n->get<SubscriberType*>(); delete n; }
  cout <<"bye bye" <<endl;
  return 0;
}
