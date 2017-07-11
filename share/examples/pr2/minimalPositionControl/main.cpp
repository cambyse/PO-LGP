#include <RosCom/roscom.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControlThread.h>
#include <Hardware/gamepad/gamepad.h>
#include <Kin/kinViewer.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {
    Access<CtrlMsg> ctrl_ref(NULL, "ctrl_ref");
    Access<CtrlMsg> ctrl_obs(NULL, "ctrl_obs");
    Access<arr>     pr2_odom(NULL, "pr2_odom");

    TaskControlThread tcm;
    GamepadInterface gamepad;
//    OrsViewer view;
    OrsPoseViewer controlview({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld);
    SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub1("/marc_rt_controller/jointState", ctrl_obs);
    PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub1("/marc_rt_controller/jointReference", ctrl_ref);
    SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub2("/robot_pose_ekf/odom_combined", pr2_odom);
    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    tcm.verbose = true;

    threadOpenModules(true);

    mlr::wait(.2);

    CtrlTask task("endeffL", new TaskMap_Default(posTMT, tcm.modelWorld.get()(), "endeffL", NoVector, "base_footprint"), 1., .8, .0, 0.);
//    task.setGains(20.,2.);
    task.map.phi(task.y, NoArr, tcm.modelWorld.get()());
    task.y_ref = task.y;
    tcm.ctrlTasks.set() = { &task };

    mlr::wait(1.);

    task.y_ref = task.y + ARR(.2, 0., 0.);

    ofstream fil("z.task");
    for(uint t=0;t<50;t++){
      fil <<mlr::realTime() <<' ' <<task.y <<endl;
      mlr::wait(.1);
    }

    task.y_ref -= ARR(.2, 0., 0.);

    mlr::wait(2.);

////    tcm.ctrlTasks.set() = {};
////    tcm.taskController->qNullCostRef.prec *= .1;
    task.prec = ARR(0., 100., 0.);
//    task.Kp = ARR(0., 20., 0.);

    moduleShutdown()->waitForStatusGreaterThan(0);

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
