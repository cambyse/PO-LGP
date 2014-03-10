#include <System/engine.h>
#include <Hardware/joystick.h>
#include <Media/audio.h>

//void threadedRun() {
//  struct MySystem:System{
//    ACCESS(arr, joystickState);
//    MySystem(){
//      addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
//      connect();
//    }
//  } S;

//  engine().open(S);

//  engine().shutdown.waitForSignal();

//  engine().close(S);
//  cout <<"bye bye" <<endl;
//}

void play(){
  JoystickInterface joy;
  SineSound S;
  Audio audio;

  createVariables(ARRAY<Module*>(&joy));

  joy.open();
  audio.open(S);
  S.addNote(880.,.2,0);
  for(uint k=0;;k++){
    MT::wait(.001);
    joy.step();
    arr s = joy.joystickState.get();
    double freq=s(4);
    S.changeFreq(0, 880.*pow(2, s(4)));
    S.notes(0,1)=.001 + .2*(s(3)+1.);
    if(MT::realTime()>10.) break;
  }

  joy.close();
  audio.close();

}

int main(int argc,char **argv){
  play();
  return 0;
};
