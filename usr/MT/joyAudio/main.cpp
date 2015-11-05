//#include <System/engine.h>
#include <Hardware/gamepad.h>
#include <Media/audio.h>

//void threadedRun() {
//  struct MySystem{
//    ACCESS(arr, gamepadState);
//    MySystem(){
//      new GamepadInterface;
//      connect();
//    }
//  } S;

//  threadOpenModules(true);

//  shutdown().waitForValueGreaterThan(0);

//  threadCloseModules();
//  cout <<"bye bye" <<endl;
//}

void play(){
  GamepadInterface gamepad;
  SineSound S;
  Audio audio;

  createVariables(mlr::Array<Module*>({&gamepad}));

  gamepad.open();
  audio.open(S);
  S.addNote(880.,.2,0);
  for(uint k=0;;k++){
    mlr::wait(.001);
    gamepad.step();
    arr s = gamepad.gamepadState.get();
    double freq=s(4);
    S.changeFreq(0, 880.*pow(2, s(4)));
    S.notes(0,1)=.001 + .2*(s(3)+1.);
    if(mlr::realTime()>10.) break;
  }

  gamepad.close();
  audio.close();

}

int main(int argc,char **argv){
  play();
  return 0;
};
