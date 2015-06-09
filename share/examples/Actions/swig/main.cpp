#include <Actions/swig.h>

#include <Core/util.h>
#include <time.h>
#include <iostream>

// void sleep(unsigned int mseconds){
//   std::cout << clock() << endl;
//   clock_t goal = mseconds + clock();
//   while (goal > clock());
//   std::cout << clock() << endl;
// }

// ============================================================================
int main(int argc, char** argv) {

  ActionSwigInterface S(false);
//  S.defineNewTaskSpaceControlAction("positionHandL",
//    { {"type","pos"}, {"ref1","endeffL"}, {"target","[.7, .3, .7]"}, {"PD","[0., 0., 0., 0.]"} } ); 0 gains makes no sense!!!???

//  S.startActivity(S.lit({"positionHandL"}));
//  S.waitForCondition(S.lit({"positionHandL", "conv"}));

  S.defineNewTaskSpaceControlAction("positionHandR", {"FollowReferenceActivity"},
  {{"type","wheels"}, {"target","[.0, .0, .5]"}, {"PD","[.5, .9, .1, 10.]"}});

  S.startActivity({"positionHandR"});
  S.waitForCondition({"conv", "positionHandR"});


  S.defineNewTaskSpaceControlAction("positionHand2", {"FollowReferenceActivity"},
    {{"type","pos"}, {"ref1","endeffL"}, {"target","[.7, .3, .9]"}, {"PD","[.5, .9, .1, 10.]"}});

  S.startActivity({"positionHand2"});
  S.waitForCondition({"conv", "positionHand2"});

//  S.createNewSymbol("quit");
//  S.waitForQuitSymbol();

  return 0;
}


