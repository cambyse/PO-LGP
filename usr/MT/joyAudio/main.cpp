#include <System/engine.h>
#include <Hardware/gamepad.h>
#include <Media/audio.h>

//void threadedRun() {
//  struct MySystem:System{
//    ACCESS(arr, gamepadState);
//    MySystem(){
//      addModule<GamepadInterface>(NULL, Module_Thread::loopWithBeat, .01);
//      connect();
//    }
//  } S;

//  engine().open(S);

//  engine().shutdown.waitForSignal();

//  engine().close(S);
//  cout <<"bye bye" <<endl;
//}

void play(){
  GamepadInterface gamepad;
  SineSound S;
  Audio audio;

  createVariables(ARRAY<Module*>(&gamepad));

  gamepad.open();
  audio.open(S);
  S.addNote(880.,.2,0);
  for(uint k=0;;k++){
    MT::wait(.001);
    gamepad.step();
    arr s = gamepad.gamepadState.get();
    double freq=s(4);
    S.changeFreq(0, 880.*pow(2, s(4)));
    S.notes(0,1)=.001 + .2*(s(3)+1.);
    if(MT::realTime()>10.) break;
  }

  gamepad.close();
  audio.close();

}

int main(int argc,char **argv){
  play();
  return 0;
};
