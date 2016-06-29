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

    while(1)
    {
      object_database.readAccess();
      FilterObjects filter_objects = object_database();
      FilterObjects alvars;
      alvars.clear();
      for (FilterObject* fo : filter_objects)
      {
          if (fo->type == FilterObject::FilterObjectType::alvar)
          {
            alvars.append(fo);
          }
      }
      if (alvars.N == 0)
      {
        std::cout << "No alvars found" << std::endl;
        mlr::wait(.05);
        object_database.deAccess();
        object_database.waitForNextRevision();
        continue;
     }
      else
      {
        cout << "Alvars found: ";
        for (uint i = 0; i < alvars.N; i++)
        {
          cout << alvars(i)->id << ' ' ;
        }
        cout << endl;
      }
      mlr::wait(.05);
      object_database.deAccess();
      object_database.waitForNextRevision();
    }
    moduleShutdown().waitForValueGreaterThan(0);

    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
