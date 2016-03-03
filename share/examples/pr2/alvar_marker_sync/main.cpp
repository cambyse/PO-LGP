//#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/rosalvar.h>
#include <Actions/TaskControllerModule.h>

// =================================================================================================
struct MySystem {
  ACCESSname(AlvarMarkers, ar_pose_markers);

  MySystem() {
    // This is for ros communication
    new RosCom_Spinner();

    // This is here to get the PR2 model
    new TaskControllerModule(); 

    // This syncs the ors graph
    new AlvarSyncer();

    // This reads the ros topic and updates the variable
    new Subscriber<AlvarMarkers>("/ar_pose_marker", (Access_typed<AlvarMarkers>&) ar_pose_markers);
  }
};


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("nodeName");
  bool useRos = true;

  MySystem system;
  threadOpenModules(true);

  for (int i = 0; true; i++) {
    mlr::wait(0.01);
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
