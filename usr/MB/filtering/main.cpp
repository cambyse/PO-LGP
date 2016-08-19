#include <Ors/ors.h>
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

  SubscribeTabletop tabletop_subscriber;
  mlr::wait(3);
  SubscribeAlvar alvar_subscriber;
  mlr::wait(3);
  Collector data_collector;
  mlr::wait(3);
  Filter myFilter;
  mlr::wait(3);
  PublishDatabase myPublisher;
  mlr::wait(3);
  RosCom_Spinner spinner;

  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);
  modulesReportCycleTimes();

  threadCloseModules();
  registry().clear();
  cout <<"bye bye" <<endl;
  return 0;
}
