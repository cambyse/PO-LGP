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

// note that this is an old version of HRI_task.cpp used for the HAI 2017 language paper

using std::cout;
using std::endl;
using std::string;

HRITask::HRITask() : R(true, false){
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
    outputs.waitForRevisionGreaterThan(rev+30);
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

  percept_filter();
  state.updateFromModelworld(R);
  cout << state << endl;
  OrsViewer viewer1("modelWorld");
  R.wait();

  auto pcl = R.PclPipeline(false);
  while (1) { // main loop
    percept_nofilter();
    performAction();
  }


}

void HRITask::performAction() {
  cout << state << endl;
  R.wait(.1); // no specific task -> just wait for user input
}
