#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>

#include <Actions/ClusterFilterActivity.h>

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  rosCheckInit("filter");

  ACCESSname(visualization_msgs::MarkerArray, tabletop_clusters)

  Subscriber<visualization_msgs::MarkerArray>  sub_markers("/tabletop/clusters", tabletop_clusters);

  ClusterFilter cf;

  RosCom_Spinner spinner;
  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);
  modulesReportCycleTimes();

  threadCloseModules();
  registry().clear();
  cout <<"bye bye" <<endl;
  return 0;
}
