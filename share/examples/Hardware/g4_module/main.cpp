#include <Hardware/G4/G4.h>
#include <Hardware/G4/module_G4Display.h>

void miniTest(){
  G4Poller g4;

  g4.open();

  for(uint i=0;i<100;i++){
    g4.step();
    mlr::wait(.001, false);
  }

  g4.close();
}


struct G4System {
  ACCESS(floatA, poses);

  G4Poller poller;
  G4Display display;
  G4System(){

    // display.mid().load("./g4mapping.kvg");
    // display.kw().init("./world.ors");

    //connect();
  }
};

void serialRun(){
  G4System S;

  openModules();
  for(uint i=0;i<100;i++){
    stepModules();
    mlr::wait(.01, false);
    cout <<i <<' ' <<S.poses.get()() <<endl;
  }
  closeModules();
}


void threadedRun(){
  G4System S;

  threadOpenModules(true);

  for(;;){
    S.poses.data->waitForNextRevision();
    cout <<S.poses.get()() <<endl;
    if(moduleShutdown()->getStatus()>0) break;
  }

  threadCloseModules();
}

int main(int argc, char **argv) {

  // miniTest();
  serialRun();
//  threadedRun();
  return 0;
}

