#include <System/engine.h>
#include <sys/time.h>

#include <Hardware/G4/G4.h>
#include <Hardware/G4/module_G4Display.h>
#include <Core/module_FloatA_Recorder.h>
#include <Hardware/G4/module_G4Recorder.h>
#include <Hardware/G4/module_G4Printer.h>

struct G4System:System {
  ACCESS(floatA, poses);
  G4System(){
    addModule<G4Poller>("POLLER", Module_Thread::loopWithBeat, .001);//8333); // 120Hz
    addModule<G4Display>("DISPLAY", Module_Thread::loopWithBeat, .033); // 30Hz
    // addModule<G4Recorder>("RECORDER", Module_Thread::listenFirst, .1);
    // addModule<G4Printer>("PRINTER", Module_Thread::loopWithBeat, 1);
    connect();
  }
};

void threadedRun(){
  G4System S;
  cout <<S <<endl;

  engine().open(S);

  engine().shutdown.waitForSignal();

  cout << "bye bye" << endl;
  engine().close(S);
}


int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  threadedRun();
  return 0;
}

