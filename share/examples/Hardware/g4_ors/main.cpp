#include <System/engine.h>
#include <sys/time.h>

#include <Hardware/G4/G4.h>
#include <Ors/module_G4Display.h>
#include <Core/module_FloatA_Recorder.h>


struct G4System:System{
  ACCESS(floatA, currentPoses);
  G4System(){
    addModule<G4Poller>("POLLER", ModuleThread::loopWithBeat, .001);//8333); // 120Hz
    addModule<G4Display>("DISPLAY", ModuleThread::loopWithBeat, .033); // 30Hz
    FloatA_Recorder *m = addModule<FloatA_Recorder>("SAVER", ModuleThread::listenFirst, .1);
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

