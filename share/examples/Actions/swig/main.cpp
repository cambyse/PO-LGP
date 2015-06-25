#include <Actions/swig.h>

#include <Core/util.h>
#include <time.h>
#include <iostream>

// ============================================================================
int main(int argc, char** argv) {

  ActionSwigInterface S;
//  S.defineNewTaskSpaceControlAction("positionHandL",
//    { {"type","pos"}, {"ref1","endeffL"}, {"target","[.7, .3, .7]"}, {"PD","[0., 0., 0., 0.]"} } ); 0 gains makes no sense!!!???

//  S.startActivity(S.lit({"positionHandL"}));
//  S.waitForCondition(S.lit({"positionHandL", "conv"}));

  //S.defineNewTaskSpaceControlAction("positionHandR", {"FollowReferenceActivity"},
  //{{"type","wheels"}, {"target","[.0, .0, .5]"}, {"PD","[.5, .9, .1, 10.]"}});

  //S.startActivity({"positionHandR"});
  //S.waitForCondition({"conv", "positionHandR"});


  //S.defineNewTaskSpaceControlAction("positionHand2", {"FollowReferenceActivity"},
   // {{"type","pos"}, {"ref1","endeffL"}, {"target","[.7, .3, .9]"}, {"PD","[.5, .9, .1, 10.]"}});

//  S.startActivity({"positionHand2"});
//  S.waitForCondition({"conv", "positionHand2"});


  S.defineNewTaskSpaceControlAction("base", {"FollowReferenceActivity"},
  {{"type","wheels"}, {"target","[.0, 0.1, 0.]"}, {"PD","[.5, .9, .1, 5.]"}});

  S.startActivity({"base"});
  S.waitForCondition({"conv", "base"});

  S.defineNewTaskSpaceControlAction("q", {"FollowReferenceActivity"},
  {{"type","qItself"},{"ref1","l_gripper_joint"}, {"target","[0.05]"}, {"PD","[2., .8, .1, 1.]"}});

  S.startActivity({"q"});
  S.waitForCondition({"conv", "q"});


//  S.createNewSymbol("quit");
//  S.waitForQuitSymbol();

  return 0;
}


