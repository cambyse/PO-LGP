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
//  FILE("machine_simple.fol") >>G;

  ItemL actions=G.getItems("Action");
  ItemL rules=G.getItems("Rule");
  ItemL state=getLiteralsOfScope(G);

  for(Item* a:actions){
    nice.append<bool>(a->keys(1), NULL, false);
  }

  for(Item* rule:rules){
    Graph& Rule=rule->kvg();
    Item *rit = nice.append<bool>(STRINGS_2("box","R"), NULL, false);
    for(Item* prec:Rule) if(prec->parents.N){
      Item *pit=nice[prec->parents(0)->keys(1)];
      if(pit){
        MT::String label/*("pre")*/;
        if(prec->parents.N>1) label <<prec->parents(1)->keys(1);
        nice.append<bool>({label}, {pit, rit}, NULL, false);
      }
    }
    Graph &effect = Rule.last()->kvg();
    for(Item* eff:effect){
      Item *pit=nice[eff->parents(0)->keys(1)];
      if(pit){
        MT::String label/*("eff")*/;
        if(eff->parents.N>1) label <<eff->parents(1)->keys(1);
        if(eff->getValueType()==typeid(bool) && eff->V<bool>()==false) label <<'!';
        nice.append<bool>({label}, {rit, pit}, NULL, false);
      }
    }

  }

  nice >>FILE("z.nice");
  nice.writeDot(FILE("z.dot").getOs(), false, true);
  system("dot -Tpdf z.dot > z.pdf");
}

// ============================================================================
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  
//  testPush();
//  testFol();
  rewriteGraph();

  return 0;
}
