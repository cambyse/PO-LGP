#include <System/engine.h>
#include <Hardware/joystick.h>

void threadedRun() {
  struct MySystem:System{
    ACCESS(arr, joystickState);
    MySystem(){
      addModule<JoystickInterface>(NULL, ModuleThread::loopWithBeat, .01);
      connect();
    }
  } S;

  engine().open(S);

  engine().shutdown.waitForSignal();

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
