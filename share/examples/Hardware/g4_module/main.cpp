#include <Hardware/G4/G4.h>
#include <System/engine.h>

void miniTest(){
  G4Poller g4;

  g4.open();

  for(uint i=0;i<100;i++){
    g4.step();
    MT::wait(.001, false);
  }

  g4.close();
}


struct G4System:System{
  ACCESS(floatA, currentPoses);
  G4System(){
    addModule("G4Poller", "G4Poller", Module_Thread::loopWithBeat, .001);
    connect();
  }
};

void serialRun(){
  G4System S;

  S.open();
  for(uint i=0;i<100;i++){
    S.step();
    MT::wait(.01, false);
    cout <<i <<' ' <<S.currentPoses.get()() <<endl;
  }
  S.close();
}


void threadedRun(){
  G4System S;

  engine().open(S);
  for(uint i=0;i<10;i++){
    S.currentPoses.var->waitForNextRevision();
    cout <<i <<' ' <<S.currentPoses.get()() <<endl;
  }
  engine().close(S);
}

int main(int argc, char **argv) {

//  miniTest();
//  serialRun();
  threadedRun();
  return 0;
}

