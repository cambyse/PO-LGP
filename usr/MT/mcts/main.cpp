#include <MCTS/mcts.h>
#include <Gui/graphview.h>

struct ToyWorld{
  int state, T;
  void resetToStart(){ state=0; T=0; }
  double advance(int d){
    state += d;
    T++;
    if(terminal()){ return (double)state/10.; }
    return 0.;
  }
  MT::Array<int> getDecisions(){ return intA({-1, 1}); }
  bool terminal(){ return T>=10; }
  double advanceRandomly(){
    if(MT::rnd.uni()<.5) return advance(+1.);
    return advance(-1.);
  }

};

int main(int argc,char **argv){
  ToyWorld world;
  MCTS<ToyWorld, int> mcts(world);
  for(uint k=0;k<10000;k++) mcts.addRollout();

  cout <<mcts.Qfunction() <<endl;

  Graph G = mcts.getGraph();
  GraphView gv(G);
  gv.watch();
}
