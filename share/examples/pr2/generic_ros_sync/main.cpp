#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>
#include <std_msgs/String.h>

//===========================================================================
ROSSUB("/hello_world", std_msgs::String, hello_world);
// This create ROSSUB_hello_world module
// Then:
//  - add the Module to your system
//  - add the ACCESS variable to your system


// =================================================================================================
struct MySystem:System {
  //  - add the ACCESS variable to your system
  ACCESS(std_msgs::String, hello_world)

  MySystem() {
    addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .01);
    //  - add the Module to your system
    addModule<ROSSUB_hello_world>(NULL, Module::listenFirst, .1);
    connect();
  }
};


// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  MySystem system;
  engine().open(system);
  cout << "Engine open " << endl;

  for (int i = 0; true; i++) {
    // print the data on the /hello_world topic
    system.hello_world.waitForNextRevision();
    std_msgs::String d = system.hello_world.get();
    cout << "step[" << i << "]: "<< d.data << endl;
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
