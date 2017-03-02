#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Control/TaskControlThread.h>
#include <Hardware/gamepad/gamepad.h>
#include <Kin/kinViewer.h>
#include <RosCom/baxter.h>

#include <RosCom/subscribeTabletop.h>
#include <RosCom/subscribeTableArray.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/perceptionCollection.h>
#include <Perception/filter.h>
#include <Perception/percept.h>
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

    ACCESSname(Percepts, percepts_filtered)
    ACCESSname(Percepts, percepts_input)

    PublishDatabase myPublisher;

    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    threadOpenModules(true);

    for(;!moduleShutdown().getStatus();){
      percepts_input.waitForNextRevision();

      uint i=0;
      cout <<"==================" <<endl;
      percepts_input.readAccess();
      for(Percept* fo : percepts_input()){
        cout <<"Object " <<i++ <<' ';
        fo->write(cout);
        cout <<endl;
      }
      percepts_input.deAccess();

      cout <<"-------------------" <<endl;
      percepts_filtered.readAccess();
      for(Percept* fo : percepts_filtered()){
        cout <<"Object " <<i++ <<' ';
        fo->write(cout);
        cout <<endl;
      }
      percepts_filtered.deAccess();
    }

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
