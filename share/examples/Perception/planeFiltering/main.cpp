#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsViewer.h>
#include <RosCom/baxter.h>

#include <RosCom/subscribeTabletop.h>
#include <RosCom/subscribeTableArray.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("planeFiltering");

  {

    OrsViewer view;
    SubscribeAlvar alvar_subscriber;
//    SubscribeTabletop tabletop_subscriber;
    SubscribeTableArray tableArray_subscriber;
    Collector data_collector;

    Filter myFilter;

    ACCESSname(FilterObjects, object_database)
    ACCESSname(FilterObjects, perceptual_inputs)

    PublishDatabase myPublisher;

    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    threadOpenModules(true);

    for(;!moduleShutdown().getValue();){
      perceptual_inputs.waitForNextRevision();

      uint i=0;
      cout <<"==================" <<endl;
      perceptual_inputs.readAccess();
      for(FilterObject* fo : perceptual_inputs()){
        cout <<"Object " <<i++ <<' ';
        fo->write(cout);
        cout <<endl;
      }
      perceptual_inputs.deAccess();

      cout <<"-------------------" <<endl;
      object_database.readAccess();
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
