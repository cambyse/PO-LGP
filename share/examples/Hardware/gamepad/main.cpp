#include <System/engine.h>
#include <Hardware/gamepad/gamepad.h>

void threadedRun() {
  struct MySystem:System{
    ACCESS(arr, gamepadState);
    MySystem(){
      addModule<GamepadInterface>(NULL, Module_Thread::loopWithBeat, .01);
      connect();
    }
  } S;

  engine().open(S);

  for(;;){
    S.gamepadState.var->waitForNextRevision();
    cout <<"\r" <<S.gamepadState.get()() <<std::flush;
    if(engine().shutdown.getValue()) break;
  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}

//void rawTest(){
//  KinectPoller kin;
//  kin.open();
//  kin.close();
//}

int main(int argc,char **argv){
  //  rawTest();
  threadedRun();
  return 0;
};
