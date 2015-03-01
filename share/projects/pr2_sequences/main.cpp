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

  activity.machine->waitForQuitSymbol();
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
