#include "PlayForceSound.h"
#include <Media/audio.h>

//uintA DUR = {0u,2,4,5,7,9,11};
extern uintA DUR;

// ============================================================================

PlayForceSoundActivity::PlayForceSoundActivity() : Thread("PlayForceSoundActivity", .1){
  threadLoop();
}

PlayForceSoundActivity::~PlayForceSoundActivity(){
  threadClose();
}

void PlayForceSoundActivity::step(){
  sound().addNote(12 + rnd(2)*12 + DUR(rnd(7)));
  sound().addNote(12 + rnd(2)*12 + DUR(rnd(7)));
}

// ============================================================================

RUN_ON_INIT_BEGIN(PlayForceSound)
registerActivity<PlayForceSoundActivity>("PlayForceSound");
RUN_ON_INIT_END(PlayForceSound)
