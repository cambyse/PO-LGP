#include <Gui/graphview.h>

#include <MCTS/solver_marc.h>
#include <MCTS/problem_BlindBranch.h>

int main(int argc,char **argv){
  BlindBranch world(10);
  MCTS mcts(world);
  Graph G = mcts.getGraph();
  GraphView gv(G);
  for(uint k=0;k<10000;k++){
    mcts.addRollout();
//    G = mcts.getGraph();
//    if(!(k%1)) gv.update();
  }

  cout <<mcts.Qfunction() <<endl;

  G = mcts.getGraph();
  gv.watch();
}
