#include <Actions/actionMachine.h>
#include <Actions/actionMachine_internal.h>
#include <Actions/actions.h>
#include <Motion/motionHeuristics.h>

#include <FOL/fol.h>


//===========================================================================
void TEST(Push) {
  ActionSystem activity;
  threadOpenModules(true);
  
  new CoreTasks(*activity.machine);

  activity.machine->waitForQuitSymbol();
  threadCloseModules();
}

// ============================================================================

void testFol(){
  Graph G;
  FILE("machine.fol") >>G;
  G.checkConsistency();
  cout <<G <<endl;

  Node *terminal=G["Terminal"]->graph()(0);
  forwardChaining_FOL(G, terminal, NoGraph, true);

}

// ============================================================================

void rewriteGraph(){
  Graph G, nice;
  FILE("machine.fol") >>G;
//  FILE("machine_simple.fol") >>G;

  NodeL actions=G.getNodes("Action");
  NodeL rules=G.getNodes("Rule");
  NodeL state=getLiteralsOfScope(G);

  for(Node* a:actions){
    nice.newNode<bool>(a->keys(1), NULL, false);
  }

  for(Node* rule:rules){
    Graph& Rule=rule->graph();
    Node *rit = nice.newNode<bool>({"box"}, NULL, false);
    for(Node* prec:Rule) if(prec->parents.N){
      Node *pit=nice[prec->parents(0)->keys(1)];
      if(pit){
        mlr::String label/*("pre")*/;
        if(prec->parents.N>1) label <<prec->parents(1)->keys(1);
        nice.newNode<bool>({label}, {pit, rit}, NULL, false);
      }
    }
    Graph &effect = Rule.last()->graph();
    for(Node* eff:effect){
      Node *pit=nice[eff->parents(0)->keys(1)];
      if(pit){
        mlr::String label/*("eff")*/;
        if(eff->parents.N>1) label <<eff->parents(1)->keys(1);
        if(eff->isOfType<bool>() && eff->get<bool>()==false) label <<'!';
        nice.newNode<bool>({label}, {rit, pit}, NULL, false);
      }
    }

  }

  nice >>FILE("z.nice");
  nice.writeDot(FILE("z.dot").getOs(), false, true);
  system("dot -Tpdf z.dot > z.pdf");
}

// ============================================================================
int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);
  
  testPush();
//  testFol();
//  rewriteGraph();

  return 0;
}
