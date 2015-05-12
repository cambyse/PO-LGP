#include <Core/graph.h>
#include <Core/registry.h>

//const char *filename="/home/mtoussai/git/3rdHand/documents/USTT/14-meeting3TUD/box.kvg";
const char *filename=NULL;

//===========================================================================

void TEST(Read){
  Graph G;

  G.checkConsistency();
  cout <<"\n** reading graph..." <<flush;
  G <<FILE(filename?filename:"example.kvg");
  G.checkConsistency();
  cout <<"\ndone" <<endl;
  cout <<"read kvg=\n--------------------\n" <<G <<"\n--------------------" <<endl;

//  Item *m = G.getItem("modify");
//  G.merge(m);
//  cout <<"'k modify' merged with 'k':" <<*G["k"] <<endl;

  G.checkConsistency();
  if(filename) return; //below only for "example.kvg"
  cout <<"\n** access to individual items:" <<endl;
  cout <<*G["k"] <<endl;
  cout <<G["k"]->kvg() <<endl;
  cout <<G["val"]->V<double>() <<endl;
  cout <<G["k"]->kvg()["z"]->V<MT::String>() <<endl;
  cout <<"DONE" <<endl;
}

//===========================================================================

void TEST(Init){
  Graph G = {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", MT::String("BLA")} };
  cout <<G <<endl;
  G.checkConsistency();
}

//===========================================================================

const Graph& rndContainer(const Graph& G){
  const Graph *g=&G;
  while(rnd.uni()<.8){
    if(!g->isItemOfParentKvg) break;
    g = &g->isItemOfParentKvg->container;
  }
  return *g;
}

Graph& rndSubgraph(Graph& G){
  Graph *g=&G;
  while(rnd.uni()<.8){
    ItemL subgraphs = g->getTypedItems<Graph>(NULL);
    if(!subgraphs.N) break;
    g = &subgraphs.rndElem()->kvg();
  }
  return *g;
}

ItemL rndParents(const Graph& G){
  if(!G.N) return {};
  uint nparents=rnd(0,10);
  ItemL par;
  for(uint i=0;i<nparents;i++){
    par.append(rndContainer(G).rndElem());
  }
  return par;
}

void rndModify(Graph& G){
  switch(rnd(4)){
    case 0://add bool item
      new Item_typed<bool>(G, {MT::String().setRandom(), MT::String().setRandom()}, rndParents(G), new bool(true), true);
      break;
    case 1://add Subgraph item
      new Item_typed<Graph>(G, {MT::String().setRandom(), MT::String().setRandom()}, rndParents(G), new Graph(), true);
      break;
    case 2://delete item
      if(G.N) delete G.rndElem();
      break;
    case 3://clone an item
      if(G.N) G.rndElem()->newClone(G);
      break;
    default:HALT("");
  }
}

void TEST(Random){
  Graph A,B;

  for(uint k=0;k<2000;k++){
    rndModify(rndSubgraph(A));

    cout <<"---" <<endl <<A <<endl;

    A.checkConsistency();
    B = A;
    B.checkConsistency();
  }
  A.clear();
  A.checkConsistency();
  B.clear();
  B.checkConsistency();
}

//===========================================================================

void TEST(Dot){
  Graph G;
  G <<FILE(filename?filename:"coffee_shop.fg");
  G.checkConsistency();
//  G.sortByDotOrder();
//  G.checkConsistency();
  G.writeDot(FILE("z.dot").getOs());
}

//===========================================================================

struct Something{
  Something(double y=0.){ x=y; }
  double x;
};
void operator<<(ostream& os, Something& s){ os <<s.x; }
//the following 2 lines are optional: they enable naming the type and typed reading from file
void operator>>(istream& is, Something& s){ is >>s.x; }
REGISTER_TYPE(Something)

void TEST(Manual){
  Graph G;
  G.append({"hallo"}, {}, new Something(3), true);
  cout <<G <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){

//  cout <<"GLOBAL LATENT REGISTRY:\n" <<registry() <<endl;

  if(argc>=2) filename=argv[1];

  testRandom();
//    testRead();
//  testInit();
//  testDot();

//  if(!filename) testManual();

  return 0;
}
