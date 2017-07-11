#include "PlayFunnySound.h"
#include <Media/audio.h>

uintA DUR = {0u,2,4,5,7,9,11};

// ============================================================================

PlayFunnySoundActivity::PlayFunnySoundActivity() : Thread("PlayFunnySoundActivity", .1){
  threadLoop();
}

PlayFunnySoundActivity::~PlayFunnySoundActivity(){
  threadClose();
}

void PlayFunnySoundActivity::step(){
  sound().addNote(12 + rnd(2)*12 + DUR(rnd(7)));
  sound().addNote(12 + rnd(2)*12 + DUR(rnd(7)));
}

// ============================================================================

RUN_ON_INIT_BEGIN(PlayFunnySound)
//registerActivity<PlayFunnySoundActivity>("PlayFunnySound");
RUN_ON_INIT_END(PlayFunnySound)
