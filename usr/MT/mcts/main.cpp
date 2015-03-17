#include <MCTS/mcts.h>
#include <Gui/graphview.h>

struct ToyWorld{
  int state, T, H;
  ToyWorld(uint H):H(H){}
  void resetToStart(){ state=0; T=0; }
  double advance(int d){
    state += d;
    T++;
    if(terminal()){ return (double)state/H; }
    return 0.;
  }
  MT::Array<int> getDecisions(){ return intA({-1, 1}); }
  bool terminal(){ return T>=H; }
  double advanceRandomly(){
    if(MT::rnd.uni()<.5) return advance(+1.);
    return advance(-1.);
  }
};

int main(int argc,char **argv){
  ToyWorld world(15);
  MCTS<ToyWorld, int> mcts(world);
  Graph G = mcts.getGraph();
  GraphView gv(G);
  for(uint k=0;k<100000;k++){
    mcts.addRollout();
//    G = mcts.getGraph();
//    if(!(k%100)) gv.update();
  }

  cout <<mcts.Qfunction() <<endl;

  G = mcts.getGraph();
  gv.watch();
}
