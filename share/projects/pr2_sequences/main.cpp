#include <Actions/actionMachine.h>
#include <Actions/actionMachine_internal.h>
#include <Actions/actions.h>
#include <Motion/motionHeuristics.h>

#include <FOL/fol.h>


//===========================================================================
void TEST(Push) {
  ActionSystem activity;
  engine().open(activity);
  
  new CoreTasks(*activity.machine);

  Action *a, *b, *c;

  c = new FollowReference(*activity.machine, "gaze",
                          new DefaultTaskMap(gazeAtTMT, *activity.machine->world, "endeffHead", NoVector, "endeffL"),
                          {}, {}, -1., .5, .9, .1, 10., 1., -1.);

  b = new FollowReference(*activity.machine, "handAlign", new DefaultTaskMap(vecTMT, *activity.machine->world, "endeffL", Vector_x),
                          {1./MT_SQRT2, 0, -1./MT_SQRT2}, {}, -1., .5, .9, .1, 10., 100.);

  a = new FollowReference(*activity.machine, "posHand", new DefaultTaskMap(posTMT, *activity.machine->world, "endeffL"),
                          {.7, .3, .7}, {}, -1., .5, .9, .1, 10.);

//  activity.machine->waitForActionCompletion(a);

  a = new FollowReference(*activity.machine, "lowHand", new DefaultTaskMap(posTMT, *activity.machine->world, "endeffL"),
                          {.7, .3, .5}, {}, -1., .5, .9, .05, 10.);

//  activity.machine->waitForActionCompletion(a);

//  activity.machine->removeAction(b);
//  activity.machine->removeAction(c);

  a = new Homing(*activity.machine, "home");
  activity.machine->waitForActionCompletion(a);

#if 0
  a = new MoveEffTo(*activity.machine, "endeffR", {.7, -.5, .6});

  b = new AlignEffTo(*activity.machine, "endeffR", {1, 0, 0.}, {0, 0, -1.});
  activity.machine->waitForActionCompletion(a);
  activity.machine->waitForActionCompletion(b);

  a = new MoveEffTo(*activity.machine, "endeffR", {.7, -.5, .6});
  b = new AlignEffTo(*activity.machine, "endeffR", {1, 0, 0.}, {0, 0, -1.});
  activity.machine->waitForActionCompletion(a);
  activity.machine->waitForActionCompletion(b);

#endif

//  cout << "waiting" << endl;
//  MT::wait(3);
//  cout << "pushing" << endl;
//  Action* push = new PushForce(*activity.machine, "endeffR", {.0, -.05, 0}/*, {0., 1., 0.}*/);
//  activity.machine->waitForActionCompletion(push);
  
  engine().close(activity);
}

// ============================================================================

void testFol(){
  KeyValueGraph G;
  FILE("machine.fol") >>G;
  G.checkConsistency();
  cout <<G <<endl;

  Item *terminal=G["Terminal"]->kvg()(0);
  forwardChaining_FOL(G, terminal, true);

}

// ============================================================================

void rewriteGraph(){
  KeyValueGraph G, nice;
  FILE("machine.fol") >>G;

  ItemL actions=G.getItems("Action");
  ItemL rules=G.getItems("Rule");
  ItemL state=getLiteralsOfScope(G);

  for(Item* a:actions){
    nice.append<bool>(a->keys(1), NULL, false);
  }

  for(Item* rule:rules){
    Graph& r=rule->kvg();
    Item *rit = nice.append<bool>(STRING("R"<<rule->index), NULL, false);
    for(Item* prec:r) if(prec->parents.N){
      Item *pit=nice[prec->parents(0)->keys(1)];
      if(pit){
        if(prec->parents.N>1) nice.append<bool>(STRINGS_1(prec->parents(1)->keys(1)), {pit, rit}, NULL, false);
        else nice.append<bool>(STRINGS_1("active"), {pit, rit}, NULL, false);
      }
    }
    Graph &effect = r.last()->kvg();
    for(Item* eff:effect){
      Item *pit=nice[eff->parents(0)->keys(1)];
      if(pit){
        if(eff->parents.N>1) nice.append<bool>(STRINGS_1(eff->parents(1)->keys(1)), {pit, rit}, NULL, false);
        else nice.append<bool>(STRINGS_1("active"), {rit, pit}, NULL, false);
      }
    }

  }

  nice >>FILE("z.nice");
}

// ============================================================================
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  
  testPush();
//  testFol();
//  rewriteGraph();

  return 0;
}
