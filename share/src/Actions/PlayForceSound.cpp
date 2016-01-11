#include "PlayForceSound.h"
#include <Media/audio.h>

//uintA DUR = {0u,2,4,5,7,9,11};
extern uintA DUR;

// ============================================================================

PlayForceSoundActivity::PlayForceSoundActivity() : Thread("PlayForceSoundActivity", .03){
  threadLoop();
}

PlayForceSoundActivity::~PlayForceSoundActivity(){
  threadClose();
}

void PlayForceSoundActivity::open(){
  sound().addNote(10, .1, 0);
  sound().addNote(0, .1, 0);
}

void PlayForceSoundActivity::step(){
  arr force = Fl.get()();
  if(!force.N) return;
  double f = length(force.sub(0,2));
  cout <<force <<' ' <<f <<endl;
  f -= 5.;
  if(f<0.) f=0.;
  sound().changeAmp(0, f/12.+0.01);
  //sound().addNote(12 + rnd(2)*12 + DUR(rnd(7)), f);

  force = Fr.get()();
  if(!force.N) return;
  f = length(force.sub(0,2));
  cout <<force <<' ' <<f <<endl;
  f -= 5.;
  if(f<0.) f=0.;
  sound().changeAmp(1, f/12.+0.01);
}

// ============================================================================

RUN_ON_INIT_BEGIN(PlayForceSound)
registerActivity<PlayForceSoundActivity>("PlayForceSound");
RUN_ON_INIT_END(PlayForceSound)
