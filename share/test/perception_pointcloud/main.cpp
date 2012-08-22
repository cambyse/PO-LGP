#include <perception/pointcloud.h>
#include <biros/control.h>
#include <JK/utils/util.h>

SET_LOG(main, DEBUG);

int main(int argc, char **argv) {
  MT::initCmdLine(argc,argv);

  srand(time(NULL));

  DEBUG_VAR(main, birosInfo().getParameter<arr>("kinect_trans_mat"));

  // Variables
  PointCloudVar kinectData3d("KinectData3D");
  PointCloudSet objectClusters("ObjectClusters");
  ObjectSet objects("Objects");
  ObjectBeliefSet filteredObjects("filteredObjects");
  Workspace<FittingJob, FittingResult> fittingWorkspace("FittingWorkspace");
  GeometricState geo;

  // Processes
  ProcessL P = newPointcloudProcesses(1);

  P(0)->threadLoop();

  new OrsView(geo.fields(0), NULL); //example for creating vi    ews directly from code 
  b::openInsideOut();

  MT::wait(100000);

}
