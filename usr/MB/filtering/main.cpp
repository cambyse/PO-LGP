#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>
#include <pr2/subscribeTabletop.h>

#include <pr2/perceptionCollection.h>

#include <Algo/perceptionFilter.h>

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  rosCheckInit("tester");

  SubscribeTabletop subscriber;

  Collector data_collector;

  Filter myFilter;

  RosCom_Spinner spinner;

  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);
  modulesReportCycleTimes();

  threadCloseModules();
  registry().clear();
  cout <<"bye bye" <<endl;
  return 0;
}
