#include <Hardware/gamepad/gamepad.h>

void threadedRun() {
  struct MySystem{
    ACCESSname(arr, gamepadState);
    GamepadInterface gamepad;
    MySystem(){
    }
  } S;

  threadOpenModules(true);

  for(;;){
    S.gamepadState.data->waitForNextRevision();
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
