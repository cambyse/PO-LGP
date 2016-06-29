#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <RosCom/baxter.h>

#include <RosCom/subscribeTabletop.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {

    OrsViewer view;
    SubscribeAlvar alvar_subscriber;
    SubscribeTabletop tabletop_subscriber;
    Collector data_collector;

    Filter myFilter;

    ACCESSname(FilterObjects, object_database)
    PublishDatabase myPublisher;

    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    threadOpenModules(true);

    for(;!moduleShutdown().getValue();){
      object_database.waitForNextRevision();
      object_database.readAccess();

      uint i=0;
      cout <<"-------------------" <<endl;
      for(FilterObject* fo : object_database()){
        cout <<"Object " <<i++ <<' ';
        fo->write(cout);
        cout <<endl;
      }

      object_database.deAccess();
    }

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
