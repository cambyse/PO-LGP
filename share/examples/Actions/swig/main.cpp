#include <Actions/swig.h>

#include <Core/util.h>

// ============================================================================
int main(int argc, char** argv) {

  ActionSwigInterface S(false);

  S.defineNewTaskSpaceControlAction("positionHand",
    {{"type","pos"}, {"ref1","endeffL"}, {"target","[.7, .3, .7]"}, {"PD","[.5, .9, .1, 10.]"}});

  S.startActivity(S.lit({"positionHand"}));
  S.waitForCondition(S.lit({"positionHand", "conv"}));


  S.defineNewTaskSpaceControlAction("positionHand2",
    {{"type","pos"}, {"ref1","endeffL"}, {"target","[.7, .3, .9]"}, {"PD","[.5, .9, .1, 10.]"}});

  S.startActivity(S.lit({"positionHand2"}));
  S.waitForCondition(S.lit({"positionHand2", "conv"}));

//  S.createNewSymbol("quit");
//  S.waitForQuitSymbol();

  return 0;
}
