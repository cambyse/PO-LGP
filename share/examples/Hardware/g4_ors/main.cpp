#include <System/engine.h>
#include <sys/time.h>

void lib_hardware_G4();
void lib_ors();
void lib_Core();


struct G4System:System{
  ACCESS(floatA, currentPoses);
  G4System(){
    addModule("G4Poller", "G4_Poller", ModuleThread::loopWithBeat, .001);
    addModule("G4Display", "G4_Display", ModuleThread::loopWithBeat, .1);
    addModule("FloatA_Recorder", "G4_Recorder", STRINGS("currentPoses"), ModuleThread::listenFirst, .1);
    connect();
  }
};


void threadedRun(){
  G4System S;
  cout <<S <<endl;

  engine().open(S);
  uint t;
  for(t=0;/*t<100*/;t++){
    if(engine().shutdown) break;
    S.currentPoses.var->waitForNextWriteAccess();
  }

  cout <<"fps = " <<t/MT::timerRead() <<endl;
  cout << "bye bye" << endl;
  engine().close(S);
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);
  lib_hardware_G4();
  lib_ors();
  lib_Core();

  threadedRun();
  return 0;
}

