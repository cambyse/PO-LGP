#include <RosCom/roscom.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <RosCom/baxter.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {

    Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");

    TaskControllerModule tcm("baxter");
    GamepadInterface gamepad;
//    OrsViewer view;
    OrsPoseViewer ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld);

    if(mlr::getParameter<bool>("useRos")){
      new SendPositionCommandsToBaxter();
      new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
    }
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
    position.y_ref = position.y + ARR(.3, 0., 0.);; //set a target

    CtrlTask align1("align",
                    new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffR", Vector_z, NULL, Vector_x),
                    1., 1., 1., 1.);
    CtrlTask align2("align",
                    new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffR", Vector_y, NULL, Vector_x),
                    1., 1., 1., 1.);

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
