#include "HRI_task.h"
#include "HRI_state.h"
#include <Roopi/roopi.h>
#include <Control/taskControl.h>
#include <Motion/komo.h>
#include <RosCom/subscribeRosKinect.h>
#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>


using std::cout;
using std::endl;
using std::string;

HRITask::HRITask() : R(true, false), state(){
  cout << "Initialize HRI Roopi Task" << endl;
  //R.collisions(true);
  R.getTaskController().lockJointGroupControl("base");
  R.newMarker("center", ARR(.75,.0,.6));
  use_ros = mlr::getParameter<int>("useRos", 0);
}

void HRITask::percept_filter() {
  {
    auto L = R.lookAt("center", .1);
    auto an = R.armsNeutral();
    R.wait({-L, -an});
    R.wait(1.);
  }
  if (use_ros!=0) {
    auto pcl = R.PclPipeline(true);
    auto filter = R.PerceptionFilter(true);
    Access<PerceptL> outputs("percepts_filtered");
    int rev=outputs.getRevision();  
    outputs.waitForRevisionGreaterThan(rev+50);
  }
}

void HRITask::percept_nofilter() {

  {
    auto L = R.lookAt("center", .1);
    auto an = R.armsNeutral();
    R.wait({-L, -an});
    R.wait(.1);
  }

  if (use_ros!=0) {
    Access<PerceptL> outputs("percepts_input");
    int rev=outputs.getRevision();  
    outputs.waitForRevisionGreaterThan(rev+3);
    
    state.updateFromPercepts(outputs.get()());
    state.updateModelworldFromState();
  }
}


void HRITask::start() {

  SubscribeRosKinect subKin; //subscription into depth and rgb images
  ImageViewer v2("kinect_rgb");
  {
    uint torsoLiftJoint=R.getK()().getJointByName("torso_lift_joint")->to->index;
    auto torsoUp =  R.newCtrlTask(new TaskMap_qItself({torsoLiftJoint}, false), {}, {.2});
    R.wait(+torsoUp);
    R.wait();
  }
  //percept_filter();
  //state.updateFromModelworld(R);
  //cout << state << endl;
  auto pcl = R.PclPipeline(false);
  {
    auto L = R.lookAt("center", .1);
    auto an = R.armsNeutral();
    R.wait({-L, -an});
    R.wait(.1);
  }
  R.wait(0.5);
  {
    Access<PerceptL> outputs("percepts_input");
    int rev=outputs.getRevision();  
    outputs.waitForRevisionGreaterThan(rev+5);
    state.initFromPercepts(outputs.get());
    state.updateModelworldFromState();
  }
  OrsViewer viewer1("modelWorld");
  percept_nofilter();
  R.wait();
  log("start");

  while (1) { // main loop
    percept_nofilter();
    if (isPlausible(state)) //if the state is not plausible, do the perception one more time
      percept_nofilter();
    std::stringstream output;
    output<< "state, " << state;
    log(output.str());
    performAction();
  }
}

void HRITask::performAction() {
  cout << state << endl;
  R.wait(); // no specific task -> just wait for user input
}

bool HRITask::isPlausible(OnTableState& s) {
  return true; //always okay
}

void HRITask::log(string s) {
  ofstream myfile;
  myfile.open ("logfile.txt", std::ios::out | std::ios::app);
  std::time_t result = std::time(0);
  myfile << result << ", ";
  myfile << s << endl;
  myfile.close();
}
