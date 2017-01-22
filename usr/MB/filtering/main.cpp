#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

#include <RosCom/subscribeTabletop.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/publishDatabase.h>

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  rosCheckInit("tester");

  SubscribeAlvar alvar_subscriber;
  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;
  RosCom_Spinner spinner;

  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);
  modulesReportCycleTimes();

  threadCloseModules();
  registry().clear();
  cout <<"bye bye" <<endl;
  return 0;
}
