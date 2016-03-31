#include <RosCom/roscom.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {
    TaskControllerModule tcm(mlr::mlrPath("data/baxter_model/baxter.ors"));
    GamepadInterface gamepad;
    OrsViewer view;
    SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub1("/marc_rt_controller/jointState", "ctrl_obs");
    PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub1("/marc_rt_controller/jointReference", "ctrl_ref");
    SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub2("/robot_pose_ekf/odom_combined", "pr2_odom");
    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

//    tcm.verbose = true;

    threadOpenModules(true);

    Access_typed<arr> ctrl_q_ref(NULL, "ctrl_q_ref");
    ctrl_q_ref.waitForRevisionGreaterThan(10); //wait a few steps (e.g. to ensure sync with real bot)

    //-- create three tasks
    CtrlTask position("endeffL", //name
                  new DefaultTaskMap(posTMT, tcm.modelWorld.get()(), "endeffL", NoVector, "base_footprint"), //map
                  1., .8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc
    position.map.phi(position.y, NoArr, tcm.modelWorld.get()()); //get the current value
    position.y_ref = position.y + ARR(.1, 0., 0.);; //set a target

    CtrlTask align1("align",
                    new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffR", Vector_z, NULL, Vector_x),
                    1., .8, 1., 1.);
    CtrlTask align2("align",
                    new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffR", Vector_y, NULL, Vector_x),
                    1., .8, 1., 1.);

    //-- tell the controller to take care of them
    tcm.ctrlTasks.set() = { &position, &align1, &align2 };


    mlr::wait(5.);
//    moduleShutdown().waitForValueGreaterThan(0);

    //-- create a homing with
    CtrlTask homing("homing",
                  new TaskMap_qItself(),
                  .5, 1., .2, 10.);
    homing.y_ref = tcm.q0;

    tcm.ctrlTasks.set() = { &homing };

    mlr::wait(5.);
//    moduleShutdown().waitForValueGreaterThan(0);

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
