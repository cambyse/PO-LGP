#include <RosCom/roscom.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <Actions/RelationalMachineModule.h>
#include <Actions/ActivitySpinnerModule.h>
#include <RosCom/serviceRAP.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {
    //-- setup a more complex 'system', mainly composed of the TaskController and the RelationalMachine
    TaskControllerModule tcm("baxter");
    RelationalMachineModule rm;
    ActivitySpinnerModule aspin;

    GamepadInterface gamepad;
    OrsPoseViewer ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld);
    SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub1("/marc_rt_controller/jointState", "ctrl_obs");
    PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub1("/marc_rt_controller/jointReference", "ctrl_ref");
    SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub2("/robot_pose_ekf/odom_combined", "pr2_odom");
    ServiceRAP rapservice;
    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    //-- ugly...
    for(Node *n:registry().getNodes("Activity")) rm.newSymbol(n->keys.last().p);
    for(ors::Shape *sh:tcm.realWorld.shapes) rm.newSymbol(sh->name.p);

    //-- run script
    threadOpenModules(true);
    rm.runScript("script.g");
//    moduleShutdown().waitForValueGreaterThan(0);
    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
