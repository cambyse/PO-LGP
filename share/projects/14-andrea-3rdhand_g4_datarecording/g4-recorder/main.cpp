//#include <System/engine.h>
#include <sys/time.h>

#include <Hardware/G4/G4.h>
#include <Hardware/G4/module_G4Display.h>
#include <Core/thread_FloatA_Recorder.h>
#include <Hardware/G4/module_G4Recorder.h>
#include <Hardware/G4/module_G4Printer.h>

struct G4System:System {
  ACCESS(floatA, poses);
  G4System(){
    addModule<G4Poller>("POLLER", /*Module::loopWithBeat,*/ .001);//8333); // 120Hz
    addModule<G4Display>("DISPLAY", /*Module::loopWithBeat,*/ .033); // 30Hz
    // addModule<G4Recorder>("RECORDER", /*Module::listenFirst,*/ .1);
    // addModule<G4Printer>("PRINTER", /*Module::loopWithBeat,*/ 1);
    //connect();
  }
};

void threadedRun(){
  G4System S;
  cout <<S <<endl;

  threadOpenModules(true);

  moduleShutdown().waitForStatusGreaterThan(0);

  cout << "bye bye" << endl;
  threadCloseModules();
}


int main(int argc, char **argv) {
  mlr::initCmdLine(argc, argv);

  threadedRun();
  return 0;
}

