#include <Actions/swig.h>

#include <Core/util.h>
#include <time.h>
#include <iostream>



// ============================================================================
int main(int argc, char** argv) {



  ActionSwigInterface S(false);
  S.defineNewTaskSpaceControlAction("positionHandL",
    {{"type","pos"}, {"ref1","endeffL"}, {"target","[.7, .3, .7]"}, {"PD","[.5, .9, .1, 10.]"}});

  S.startActivity(S.lit({"positionHandL"}));
  S.waitForCondition(S.lit({"positionHandL", "conv"}));

  // S.defineNewTaskSpaceControlAction("positionHandR",
  // {{"type","pos"}, {"ref1","endeffR"}, {"target","[.2, .4, .6]"}, {"PD","[.5, .9, .1, 10.]"}});

  // S.startActivity(S.lit({"positionHandR"}));
  // S.waitForCondition(S.lit({"positionHandR", "conv"}));

//  S.createNewSymbol("quit");
//  S.waitForQuitSymbol();

  return 0;
}


