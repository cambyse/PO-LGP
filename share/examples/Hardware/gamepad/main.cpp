#include <Hardware/gamepad/gamepad.h>

void threadedRun() {
  struct MySystem{
    ACCESS(arr, gamepadState);
    GamepadInterface gamepad;
    MySystem(){
    }
  } S;

  threadOpenModules(true);

  for(;;){
    S.gamepadState.var->waitForNextRevision();
    cout <<"\r" <<S.gamepadState.get()() <<std::flush;
    if(moduleShutdown().getValue()) break;
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;
}

//void rawTest(){
//  KinectThread kin;
//  kin.open();
//  kin.close();
//}

int main(int argc,char **argv){
  //  rawTest();
  threadedRun();
  return 0;
};
