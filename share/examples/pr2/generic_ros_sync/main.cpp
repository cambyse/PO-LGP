//#include <System/engine.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
//#include <RosCom/rosmacro.h>
#include <std_msgs/String.h>



// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("nodeName");

  ACCESSname(std_msgs::String, hello_world)
  RosCom_Spinner spinner;
  //addModule<ROSSUB_hello_world>(NULL, /*Module::listenFirst,*/ .1);
  Subscriber<std_msgs::String> sub("/hello_world", hello_world);

  threadOpenModules(true);
  cout << "Engine open " << endl;

  for (int i = 0; true; i++) {
    // print the data on the /hello_world topic
    hello_world.waitForNextRevision();
    std_msgs::String d = hello_world.get();
    cout << "step[" << i << "]: "<< d.data << endl;
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
