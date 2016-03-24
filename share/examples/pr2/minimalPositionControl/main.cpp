#include <pr2/roscom.h>
#include <Actions/gamepadControl.h>
#include <Actions/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {
    Access_typed<CtrlMsg> ctrl_ref(NULL, "ctrl_ref");
    Access_typed<CtrlMsg> ctrl_obs(NULL, "ctrl_obs");
    Access_typed<arr>     pr2_odom(NULL, "pr2_odom");

    TaskControllerModule tcm;
    GamepadInterface gamepad;
    OrsViewer view;
    SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub1("/marc_rt_controller/jointState", ctrl_obs);
    PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub1("/marc_rt_controller/jointReference", ctrl_ref);
    SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub2("/robot_pose_ekf/odom_combined", pr2_odom);
    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    tcm.verbose = true;

    threadOpenModules(true);

    CtrlTask task("endeffL", new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffL", NoVector, "base_footprint"), 1., .8, 1., 1.);
//    task.setGains(10.,1.);
    task.map.phi(task.y, NoArr, tcm.modelWorld.get()());
    task.y_ref = task.y;
    tcm.ctrlTasks.set() = { &task };

    mlr::wait(2.);

    task.y_ref = task.y + ARR(.1, 0., 0.);

    mlr::wait(2.);

    task.y_ref -= ARR(.1, 0., 0.);

    mlr::wait(2.);

//    tcm.ctrlTasks.set() = {};
//    tcm.taskController->qNullCostRef.prec *= .1;
    task.prec = ARR(0., 0., 100.);
    task.Kp = ARR(0., 0., 10.);

    moduleShutdown().waitForValueGreaterThan(0);

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
