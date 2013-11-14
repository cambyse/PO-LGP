#include <System/engine.h>
#include <sys/time.h>

#include <Hardware/G4/G4.h>
#include <Ors/module_G4Display.h>
#include <Core/module_FloatA_Recorder.h>


struct G4System:System{
  ACCESS(floatA, currentPoses);
  G4System(){
    addModule<G4Poller>("Poller", ModuleThread::loopWithBeat, .001);
    addModule<G4Display>("Display", ModuleThread::loopWithBeat, .1);
    FloatA_Recorder *m = addModule<FloatA_Recorder>("Recorder", ModuleThread::listenFirst, .1);
    connect(m->x, "currentPoses");
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

